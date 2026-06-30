import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import sys
import os
import glob

# ===== หา log ล่าสุดอัตโนมัติ =====
def find_latest_log():
    logs = glob.glob(os.path.expanduser("~/senior/capstone_turtlebot/logs/*.csv"))
    if not logs:
        print(" ไม่พบ log file ใน logs/")
        sys.exit(1)
    latest = max(logs, key=os.path.getmtime)
    print(f" ใช้ไฟล์: {latest}")
    return latest

# ===== รับ argument หรือใช้ล่าสุด =====
if len(sys.argv) > 1:
    log_path = sys.argv[1]
else:
    log_path = find_latest_log()

# ===== โหลด CSV =====
df = pd.read_csv(log_path)
t = df["t"].values
# แปลงหน่วย เมตร เป็น เซนติเมตร ทันที
speed_r = df["speed_r"].values
speed_l = df["speed_l"].values
target_r = df["target_r"].values
# ตรวจสอบว่ามีคอลัมน์ dist หรือยัง
if "dist_r" in df.columns:
    dist_r = df["dist_r"].values
    dist_l = df["dist_l"].values
else:
    # ถ้ายังไม่มี (ไฟล์เก่าที่เละๆ) ให้คำนวณหลอกๆ ไว้ก่อน
    dist_r = t * 0
    dist_l = t * 0

# ===== Plot =====
fig = plt.figure(figsize=(12, 8))
fig.suptitle(f"TankBot Analysis - {os.path.basename(log_path)}", fontsize=14, fontweight='bold')
gs = gridspec.GridSpec(2, 1, hspace=0.4)

# --- Graph 1: Speed vs Time (cm/s) ---
ax1 = fig.add_subplot(gs[0])
ax1.plot(t, speed_r, color="royalblue", label="Actual Right", alpha=0)
ax1.plot(t, speed_l, color="tomato", label="Actual Left", alpha=0)
ax1.plot(t, target_r, color="black", linestyle="--", label="Target", linewidth=2)
ax1.set_ylabel("Speed (cm/s)")
ax1.set_title("Velocity Control Performance")
ax1.legend(loc="upper right")
ax1.grid(True, alpha=0.3)

# --- Graph 2: Distance vs Time (cm) ---
ax2 = fig.add_subplot(gs[1])
ax2.plot(t, dist_r, color="royalblue", label="Dist Right")
ax2.plot(t, dist_l, color="tomato", label="Dist Left")
ax2.set_ylabel("Distance (cm)")
ax2.set_xlabel("Time (seconds)")
ax2.set_title("Odometry Tracking")
ax2.legend(loc="upper left")
ax2.grid(True, alpha=0.3)

# บันทึกภาพ
save_path = log_path.replace(".csv", "_graph.png")
plt.savefig(save_path, dpi=150)
print(f"📊 กราฟถูกสร้างแล้วที่: {save_path}")
plt.show()
