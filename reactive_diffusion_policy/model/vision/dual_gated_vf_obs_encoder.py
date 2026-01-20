import torch
import torch.nn as nn
import torch.nn.functional as F

class ForceTokenEncoder(nn.Module):
    def __init__(self, d_model=512, in_dim=6, L=48):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv1d(in_dim, d_model, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv1d(d_model, d_model, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
        )
    def forward(self, wrench_hist):  # [B, L, 6]
        x = wrench_hist.transpose(1,2)          # [B, 6, L]
        x = self.conv(x)                        # [B, 512, L]
        x = x.transpose(1,2).contiguous()       # [B, L, 512]
        return x

class DualGatedCrossAttn(nn.Module):
    def __init__(self, d_model=512, num_heads=8, num_events=6, g_min=0.05):
        super().__init__()
        assert d_model % num_heads == 0
        self.d_model = d_model
        self.h = num_heads
        self.dh = d_model // num_heads
        self.g_min = g_min

        self.ln_q = nn.LayerNorm(d_model)
        self.ln_kv = nn.LayerNorm(d_model)

        self.q_proj = nn.Linear(d_model, d_model, bias=False)
        self.k_proj = nn.Linear(d_model, d_model, bias=False)
        self.v_proj = nn.Linear(d_model, d_model, bias=False)
        self.out_proj = nn.Linear(d_model, d_model, bias=False)

        # head gate MLP: p(event)->[H]
        self.head_gate = nn.Sequential(
            nn.Linear(num_events, 32), nn.ReLU(inplace=True),
            nn.Linear(32, num_heads)
        )

        # out_proj zero init: 初始退化≈DP
        nn.init.zeros_(self.out_proj.weight)

    def forward(self, v_feat, f_tokens, p_event, g_scalar):
        """
        v_feat: [B, 512] (single token)
        f_tokens: [B, L, 512]
        p_event: [B, 6]  softmax prob
        g_scalar: [B, 1] global gate in [0,1]
        """
        B = v_feat.shape[0]
        V = v_feat[:, None, :]                  # [B,1,512]
        Q_in = self.ln_q(V)
        KV_in = self.ln_kv(f_tokens)

        # head-wise gate
        gh = torch.sigmoid(self.head_gate(p_event))  # [B,H]

        Q = self.q_proj(Q_in).view(B,1,self.h,self.dh).transpose(1,2)     # [B,H,1,dh]
        K = self.k_proj(KV_in).view(B,-1,self.h,self.dh).transpose(1,2)   # [B,H,L,dh]
        Vv= self.v_proj(KV_in).view(B,-1,self.h,self.dh).transpose(1,2)   # [B,H,L,dh]

        attn = (Q @ K.transpose(-2,-1)) / (self.dh ** 0.5)                # [B,H,1,L]
        attn = torch.softmax(attn, dim=-1)

        out = attn @ Vv                                                  # [B,H,1,dh]
        out = out * gh[:, :, None, None]                                  # head gate
        out = out.transpose(1,2).contiguous().view(B,1,self.d_model)      # [B,1,512]
        out = self.out_proj(out)                                          # [B,1,512]

        g = torch.clamp(g_scalar, self.g_min, 1.0)                        # [B,1]
        V_fused = V + out * g[:, None, :]                                 # residual + global gate
        return V_fused[:,0,:]                                             # [B,512]

class DualGatedVFOBSencoder(nn.Module):
    """
    Wrapper: base vision obs_encoder + (PAEP -> gates -> cross-attn with force tokens)
    Output shape stays [B,512] to keep DP/RDP global_cond logic unchanged.
    """
    def __init__(self, base_obs_encoder, paep_model, force_L=48, d_model=512, num_heads=8,
                 ema_alpha=0.85, contact_ids=(1,2,3,4)):
        super().__init__()
        self.base = base_obs_encoder
        self.paep = paep_model
        for p in self.paep.parameters():
            p.requires_grad_(False)
        self.paep.eval()

        self.force_enc = ForceTokenEncoder(d_model=d_model, in_dim=6, L=force_L)
        self.fuse = DualGatedCrossAttn(d_model=d_model, num_heads=num_heads, num_events=6)

        self.ema_alpha = ema_alpha
        self.contact_ids = contact_ids
        self.register_buffer("_p_ema", torch.zeros(6), persistent=False)

    def output_shape(self):
        return self.base.output_shape()  # should be (512,)

    @torch.no_grad()
    def _paep_probs(self, img_dict, wrench_hist, tcp_pose):
        logits = self.paep(img_dict, wrench_hist, tcp_pose)  # 你按自己 PAEP forward 接口适配
        p = torch.softmax(logits, dim=-1)
        return p

    def forward(self, obs):
        """
        obs needs:
          - image keys the base encoder expects
          - wrench_hist: [B, L, 6]
          - tcp_pose:    [B, P]  (for PAEP)
        """
        # 1) vision feature from upstream encoder
        v_feat = self.base(obs)                                 # [B,512]

        # 2) force tokens
        wrench_hist = obs["wrench_hist"]                         # [B,L,6]
        f_tokens = self.force_enc(wrench_hist)                   # [B,L,512]

        # 3) PAEP prob (event distribution)
        #    这里的 img_dict/tcp_pose 取决于你 PAEP 的输入形式；如果 PAEP 直接吃 obs，也可直接传 obs
        tcp_pose = obs["tcp_pose"]                               # [B,P]
        p_event = self._paep_probs(obs, wrench_hist, tcp_pose)   # [B,6]

        # 4) EMA smooth on probs (streaming 更稳；训练时也可关掉或改成 batch EMA)
        if self.training:
            p_smooth = p_event
        else:
            self._p_ema = self.ema_alpha * self._p_ema + (1 - self.ema_alpha) * p_event.mean(dim=0)
            p_smooth = p_event * 0 + self._p_ema[None, :]

        # 5) global gate = sum contact probs
        g = p_smooth[:, self.contact_ids].sum(dim=-1, keepdim=True)        # [B,1]

        # 6) fusion
        v_fused = self.fuse(v_feat, f_tokens, p_smooth, g)                 # [B,512]
        return v_fused
