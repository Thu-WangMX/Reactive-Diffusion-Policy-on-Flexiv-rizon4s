import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from scipy.ndimage import label, binary_closing

# ================= 配置区域 =================
LOG_ROOT = Path("/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs")

EXPERIMENTS = {
    # 请确认这里的路径是你刚才录制成功的那个文件夹
    "Baseline (Image DP)": LOG_ROOT / "wmx_real_image_dp_absolute_12fps/20260123_183409",
    # "PAEP-Gated": LOG_ROOT / "..." 
}

CONTACT_THRESHOLD = 2.0
MAX_GAP_TOLERANCE = 1.0 
MIN_ACTION_DURATION = 2.0 

plt.style.use('seaborn-v0_8-whitegrid')
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False
# ===========================================

def get_fps_from_name(path_str):
    if "24fps" in path_str: return 24.0
    if "12fps" in path_str: return 12.0
    return 10.0

def load_data(exp_path):
    path = Path(exp_path)
    csv_files = sorted(list(path.glob("*fz.csv")))
    npy_files = sorted(list(path.glob("*fz.npy")))
    
    data = None
    if csv_files:
        try:
            # 自动识别 header
            df = pd.read_csv(csv_files[0])
            # 统一列名小写
            df.columns = [str(c).lower().strip() for c in df.columns]
            
            if 'fz' in df.columns:
                data = df['fz'].values
            elif len(df.columns) >= 3:
                # 备用：取第3列
                data = df.iloc[:, 2].values
            else:
                print(f"[Error] 无法识别 Fz 列: {csv_files[0]}")
                return None, 0.0

            # 确保转为 float，遇到非数字变成 NaN 然后丢弃
            data = pd.to_numeric(data, errors='coerce')
            data = data[~np.isnan(data)]

        except Exception as e:
            print(f"Error loading CSV {csv_files[0]}: {e}")
            return None, 0.0
            
    elif npy_files:
        data = np.load(npy_files[0]).flatten()
    
    if data is None or len(data) == 0:
        return None, 0.0
    return data, get_fps_from_name(str(exp_path))

def find_main_wiping_segment(force_array, fps):
    is_contact_raw = np.abs(force_array) > CONTACT_THRESHOLD
    gap_pixels = int(MAX_GAP_TOLERANCE * fps)
    structure = np.ones(gap_pixels)
    is_contact_closed = binary_closing(is_contact_raw, structure=structure)
    labeled, n_components = label(is_contact_closed)
    
    best_segment = None
    max_duration = 0
    
    for i in range(1, n_components + 1):
        idxs = np.where(labeled == i)[0]
        if len(idxs) == 0: continue
        start, end = idxs[0], idxs[-1]
        duration = (end - start) / fps
        if duration > MIN_ACTION_DURATION:
            if duration > max_duration:
                max_duration = duration
                best_segment = (start, end)
    return best_segment

def calculate_metrics(force_array, fps, name):
    abs_force = np.abs(force_array)
    segment = find_main_wiping_segment(force_array, fps)
    metrics = {"Name": name}
    
    if segment is None:
        print(f"[{name}] 未检测到有效擦拭阶段")
        metrics.update({"Mean Force (N)": 0, "Max Force (N)": 0, "Contact Ratio (%)": 0, "Avg Recovery Time (s)": 0})
        return metrics, np.array([]), np.array([])
    
    start_idx, end_idx = segment
    margin = int(0.2 * fps) 
    if end_idx - start_idx > 2 * margin:
        start_idx += margin; end_idx -= margin

    roi_force_abs = abs_force[start_idx:end_idx]
    roi_force_raw = force_array[start_idx:end_idx]
    roi_time = np.arange(start_idx, end_idx) / fps
    roi_time = roi_time - roi_time[0]
    
    is_contact_roi = roi_force_abs > CONTACT_THRESHOLD
    
    if np.any(is_contact_roi):
        contact_forces = roi_force_abs[is_contact_roi]
        metrics["Mean Force (N)"] = np.mean(contact_forces)
        metrics["Max Force (N)"] = np.max(contact_forces)
        metrics["Force Std Dev (N)"] = np.std(contact_forces)
    else:
        metrics.update({"Mean Force (N)": 0, "Max Force (N)": 0, "Force Std Dev (N)": 0})
        
    metrics["Contact Ratio (%)"] = (np.sum(is_contact_roi) / len(roi_force_abs)) * 100
    
    structure = np.ones(3, dtype=int)
    labeled_loss, n_loss = label(is_contact_roi == 0, structure)
    loss_durations = [np.sum(labeled_loss == i)/fps for i in range(1, n_loss + 1) if (np.sum(labeled_loss == i)/fps > 0.05)]
    metrics["Avg Recovery Time (s)"] = np.mean(loss_durations) if loss_durations else 0.0

    return metrics, roi_time, roi_force_raw

def main():
    print("=== 开始分析 Fz 数据 (V4 Fix) ===")
    results = []
    plot_data = []
    
    for name, path in EXPERIMENTS.items():
        if not path.exists():
            print(f"Path not found: {path}")
            continue   
        raw_data, fps = load_data(path)
        if raw_data is None: continue
        
        metrics, t, f = calculate_metrics(raw_data, fps, name)
        results.append(metrics)
        if len(t) > 0:
            plot_data.append({"name": name, "time": t, "force": f})

    if not results:
        print("无有效数据。")
        return

    df_res = pd.DataFrame(results)
    cols = ["Name", "Max Force (N)", "Mean Force (N)", "Force Std Dev (N)", "Contact Ratio (%)", "Avg Recovery Time (s)"]
    print("\n=== 分析结果 ===")
    print(df_res[cols].to_string(index=False))
    
    # 绘图逻辑
    fig = plt.figure(figsize=(16, 8))
    gs = fig.add_gridspec(2, 4)

    ax1 = fig.add_subplot(gs[0, :2])
    for item in plot_data:
        ax1.plot(item["time"], item["force"], label=item["name"], alpha=0.8)
    ax1.set_title("Wiping Phase: Force Profile"); ax1.legend(); ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(gs[0, 2])
    sns.barplot(x="Name", y="Max Force (N)", data=df_res, ax=ax2, palette="Reds")
    ax2.set_title("Max Force (Risk)"); ax2.set_xlabel(""); ax2.set_xticklabels(ax2.get_xticklabels(), rotation=15)

    ax3 = fig.add_subplot(gs[0, 3])
    sns.barplot(x="Name", y="Contact Ratio (%)", data=df_res, ax=ax3, palette="Greens")
    ax3.set_title("Contact Ratio"); ax3.set_xlabel(""); ax3.set_xticklabels(ax3.get_xticklabels(), rotation=15)

    ax4 = fig.add_subplot(gs[1, :])
    box_data = [np.abs(item["force"])[np.abs(item["force"]) > CONTACT_THRESHOLD] for item in plot_data]
    box_labels = [item["name"] for item in plot_data]
    if box_data:
        ax4.boxplot(box_data, labels=box_labels, patch_artist=True, vert=False)
        ax4.set_title("Force Distribution during Contact")

    plt.tight_layout()
    save_path = LOG_ROOT / "wiping_analysis_v4.png"
    plt.savefig(save_path)
    print(f"\n图表已保存至: {save_path}")
    plt.show()

if __name__ == "__main__":
    main()