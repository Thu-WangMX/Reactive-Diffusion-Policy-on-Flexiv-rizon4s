import zarr
import numpy as np
import os

def analyze_wiping_force_auto(zarr_path):
    print(f"Loading Zarr from: {zarr_path}")
    
    if not os.path.exists(zarr_path):
        print(f"Error: Path '{zarr_path}' does not exist.")
        return

    try:
        # 打开 Zarr 根组
        root = zarr.open(zarr_path, mode='r')
        
        # ==========================================
        # 1. 自动定位数据位置 (Root 还是 Data组?)
        # ==========================================
        data_group = None
        
        # 检查根目录是否有 paep_contact
        if 'paep_contact' in root:
            print("Found arrays in ROOT group.")
            data_group = root
        # 检查是否存在 data 组且包含 paep_contact
        elif 'data' in root and 'paep_contact' in root['data']:
            print("Found arrays in 'data' subgroup.")
            data_group = root['data']
        else:
            # 没找到，打印所有键以供调试
            print("\n[ERROR] Could not find 'paep_contact' array.")
            print(f"Keys in Root: {list(root.keys())}")
            if 'data' in root:
                print(f"Keys in 'data' group: {list(root['data'].keys())}")
            return

        # ==========================================
        # 2. 加载数据
        # ==========================================
        contacts = data_group['paep_contact'][:]
        wrenches = data_group['left_robot_tcp_wrench'][:]
        
        # 寻找 episode_ends
        episode_ends = None
        # 优先去 root['meta'] 找
        if 'meta' in root and 'episode_ends' in root['meta']:
            episode_ends = root['meta']['episode_ends'][:]
            print("Loaded episode_ends from 'meta/episode_ends'")
        # 其次去 root 找
        elif 'episode_ends' in root:
            episode_ends = root['episode_ends'][:]
            print("Loaded episode_ends from 'root/episode_ends'")
        else:
            print("[ERROR] Could not find 'episode_ends'.")
            return
            
        print(f"Total data points: {len(contacts)}")
        print(f"Total episodes: {len(episode_ends)}")
        
    except Exception as e:
        print(f"Error accessing Zarr structure: {e}")
        return

    print("=" * 80)
    print(f"{'Ep ID':<8} | {'Range (Start-End)':<20} | {'Contact Steps':<14} | {'Mean Normal Force (N)':<22}")
    print("-" * 80)

    all_ep_means = []
    start_idx = 0
    CONTACT_THRESHOLD = 0.7  # 阈值

    # ==========================================
    # 3. 统计计算
    # ==========================================
    for i, end_idx in enumerate(episode_ends):
        # 确保索引不越界
        if end_idx > len(contacts):
            print(f"Warning: episode_end {end_idx} exceeds data length {len(contacts)}. Clipping.")
            end_idx = len(contacts)

        ep_contacts = contacts[start_idx:end_idx]
        ep_wrenches = wrenches[start_idx:end_idx]
        
        # 筛选接触阶段
        contact_mask = ep_contacts > CONTACT_THRESHOLD
        contact_steps = np.sum(contact_mask)
        
        if contact_steps == 0:
            print(f"{i:<8} | {start_idx}-{end_idx:<19} | {'0':<14} | {'N/A'}")
        else:
            valid_wrenches = ep_wrenches[contact_mask]
            # 计算 Z 轴绝对值
            fz_values = np.abs(valid_wrenches[:, 2])
            mean_fz = np.mean(fz_values)
            all_ep_means.append(mean_fz)
            
            print(f"{i:<8} | {start_idx}-{end_idx:<19} | {contact_steps:<14} | {mean_fz:.4f}")
        
        start_idx = end_idx

    print("=" * 80)
    if all_ep_means:
        print(f"Summary:")
        print(f"  Valid Episodes: {len(all_ep_means)} / {len(episode_ends)}")
        print(f"  Overall Average Normal Force: {np.mean(all_ep_means):.4f} N")
    else:
        print("No valid contact phases found.")

if __name__ == "__main__":
    zarr_file_path = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr'
    analyze_wiping_force_auto(zarr_file_path)