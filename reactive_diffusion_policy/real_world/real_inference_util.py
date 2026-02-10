from typing import Dict, Callable, Tuple
import numpy as np
from reactive_diffusion_policy.common.cv2_util import get_image_transform

from typing import Dict, Tuple
import numpy as np
from reactive_diffusion_policy.common.cv2_util import get_image_transform

def get_real_obs_dict(
        env_obs: Dict[str, np.ndarray],
        shape_meta: dict,
        is_extended_obs: bool = False,
        bgr_to_rgb: bool = False,   # <-- 新增：runner 可控
        ) -> Dict[str, np.ndarray]:
    obs_dict_np = dict()
    obs_shape_meta = shape_meta['extended_obs'] if is_extended_obs else shape_meta['obs']

    for key, attr in obs_shape_meta.items():
        type = attr.get('type', 'low_dim')
        shape = attr.get('shape')

        if type == 'rgb':
            if key not in env_obs:
                raise KeyError(f"[get_real_obs_dict] Missing key '{key}' in env_obs. "
                            f"Available keys: {sorted(env_obs.keys())}")
            this_imgs_in = env_obs[key]
          # THWC
            t, hi, wi, ci = this_imgs_in.shape
            co, ho, wo = shape
            assert ci == co

            out_imgs = this_imgs_in

            # 需要 transform 的条件：分辨率不一致 OR dtype uint8 OR 需要 bgr->rgb
            need_tf = (ho != hi) or (wo != wi) or (this_imgs_in.dtype == np.uint8) or bgr_to_rgb

            if need_tf:
                tf = get_image_transform(
                    input_res=(wi, hi),
                    output_res=(wo, ho),
                    bgr_to_rgb=bgr_to_rgb,        # <-- 关键：真正启用转色
                )
                out_imgs = np.stack([tf(x) for x in this_imgs_in])

                # 保持原逻辑：uint8 才 /255
                if this_imgs_in.dtype == np.uint8:
                    out_imgs = out_imgs.astype(np.float32) / 255.0

            # THWC -> TCHW
            obs_dict_np[key] = np.moveaxis(out_imgs, -1, 1)

        elif type == 'low_dim':
            if "wrt" in key:
                continue
            this_data_in = env_obs[key]
            if 'pose' in key and shape == (2,):
                this_data_in = this_data_in[..., [0, 1]]
            obs_dict_np[key] = this_data_in

    return obs_dict_np



def get_real_obs_resolution(
        shape_meta: dict
        ) -> Tuple[int, int]:
    out_res = None
    obs_shape_meta = shape_meta['obs']
    for key, attr in obs_shape_meta.items():
        type = attr.get('type', 'low_dim')
        shape = attr.get('shape')
        if type == 'rgb':
            co,ho,wo = shape
            if out_res is None:
                out_res = (wo, ho)
            assert out_res == (wo, ho)
    return out_res
