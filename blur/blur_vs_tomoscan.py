import numpy as np
import matplotlib.pyplot as plt

# This Python script analyzes and visualizes the relationship between rotation speed, exposure time, 
# and scanning parameters for a tomographic imaging setup. 
# Two scenarios are considered:
#   1. A maximum-speed scenario constrained by a fixed detector blur limit.
#   2. A typical TomoScan scenario with a fixed angular step per projection.
# For each case, the script computes the maximum allowable rotation speed and the corresponding 
# total scan time for a 180° rotation. 
# It also accounts for the detector readout time and compares both scenarios to help plan 
# tomographic scans that balance speed and image quality.

# In both cases projections are acquired sequentially, with a minimal temporal gap equal to the detector 
# readout time between consecutive exposures.
# ##############################################
# Case 1: Maximum-speed scenario constrained by a fixed detector blur limit.
# ##############################################
detector_x_size = 2048  # horizontal detector size in pixels
r = detector_x_size / 2  # detector radius in pixels

# Exposure times (s) to evaluate
exposure_times = np.arange(0.05, 0.5, 0.05)

# Maximum allowed detector blur (pixels) for the max-speed scenario
# set_blur = 0.0022453 # pixels seelct this to match tomoScan
# set_blur = 2*1.0723301 # pixels
set_blur = 3 # pixels

# ------------------------
# Calculate maximum allowable rotation based on blur limit
# ------------------------
theta_exposure_time = np.degrees(2*np.arcsin((set_blur/(2*r)) )) # angular displacement during exposure
speeds = theta_exposure_time / exposure_times                    # max rotation speed limited by blur

# Detector readout time (s)
# frame_rate_with_zero_exposure_time = 160000  # Hz pixels select this to match tomoScan
frame_rate_with_zero_exposure_time = 16  # Hz this should be measured for each detector configuration
readout_time = 1 / frame_rate_with_zero_exposure_time

theta_readouts  = speeds * readout_time                   # angular displacement during readout
theta_per_frame = theta_exposure_time + theta_readouts    # total angular step per frame
times_per_frame = exposure_times + readout_time           # total time per frame

# ------------------------
# Total number of frames for a 180° rotation
# ------------------------
frames_per_180deg = 180 / theta_per_frame

# ##############################################
# Case 2: Typical TomoScan scenario with a fixed angular step per projection and user-defined angular step per projection
# ##############################################
rotation_step = 0.12 # degrees per projection (typical TomoScan step)
motor_speeds = rotation_step / (exposure_times + readout_time) # rotation speed required to match the step and detector readout

# Total number of projections for a 180° rotation at TomoScan speed
N_proj_180 = int(np.round(180 / rotation_step))

# Convert readout time to milliseconds for plotting
readout_time_ms = readout_time * 1000

# Compute effective blur for TomoScan fly scan (depends on speed)
effective_blur_rad = np.radians(motor_speeds * exposure_times)  # angular displacement during exposure
# effective_blur_px = r * (1 - np.cos(effective_blur_rad))
effective_blur_px = 2 * r * np.sin(effective_blur_rad/2)

# === Create figure with two stacked subplots ===
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

# --- Top plot: Angular Speeds for both scenarios ---
ax1.plot(exposure_times, speeds, 'o-', linewidth=2,
          label=f'At max speed with  blur={set_blur} px, readout={readout_time_ms:.2f} ms, projections = N')
ax1.plot(exposure_times, motor_speeds, 's-', linewidth=2,
          label=f'At TomoScan scan speed, step={rotation_step}°, readout={readout_time_ms:.4f} ms, projections={N_proj_180}, blur≈{effective_blur_px[0]:.7f}-{effective_blur_px[-1]:.7f}px')

indices = [0, len(exposure_times)//2, -1]
for i in indices:
    ax1.annotate(f'frames per 180°={int(frames_per_180deg[i])}',
                 xy=(exposure_times[i], speeds[i]),
                 xytext=(5, 10),
                 textcoords='offset points',
                 arrowprops=dict(arrowstyle='->', color='red'),
                 color='blue')

ax1.set_xlabel('Exposure Time (s)')
ax1.set_ylabel('Speed (°/s)')
ax1.set_title('Rotation speed vs Exposure Time')
ax1.grid(True)
ax1.legend(fontsize=7)  # smaller font to fit nicely

# --- Bottom plot: Total Scan Time for 180° rotation ---
total_time_min = 180 / speeds
total_time_motor = 180 / motor_speeds

ax2.plot(exposure_times, total_time_min, 'o-', linewidth=2,
          label=f'Scan time at max speed with blur={set_blur}px, readout={readout_time_ms:.2f}ms, projections=N')
ax2.plot(exposure_times, total_time_motor, 's-', linewidth=2,
          label=f'TomoScan time, step={rotation_step}°, readout={readout_time_ms:.2f}ms, projections = {N_proj_180}, blur≈blur≈{effective_blur_px[0]:.7f}-{effective_blur_px[-1]:.7f}px')

for i in indices:
    ax2.annotate(f'{total_time_min[i]:.1f}s\nN={int(frames_per_180deg[i])}',
                 xy=(exposure_times[i], total_time_min[i]),
                 xytext=(0, 15),
                 textcoords='offset points',
                 ha='center',
                 arrowprops=dict(arrowstyle='->', color='red'),
                 color='blue',
                 fontsize=10)

ax2.set_xlabel('Exposure Time (s)')
ax2.set_ylabel('Total Scan Time for 180° (s)')
ax2.set_title('Total Scan Time vs Exposure Time')
ax2.grid(True)
ax2.legend(fontsize=7)  # smaller font to fit nicely

plt.tight_layout()
plt.show()
