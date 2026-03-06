import numpy as np
import matplotlib.pyplot as plt

# Parameters
detector_x_size = 2048  # horizontal detector size in pixels
r = detector_x_size / 2

# Exposure time (s) from 0.01 to 0.5 in 0.01 s steps
exposure_times = np.arange(0.05, 0.5, 0.05)

# Set the maximum blur error acceptable
set_blur = 3  # in pixels  

# Detector readout time (s)
frame_rate_with_zero_exposure_time = 16 # Hz this should be measured for each detector configuration
readout_time = 1 / frame_rate_with_zero_exposure_time

# Compute max angular speed (°/s)
theta_exposure_time = np.degrees(2*np.arcsin((set_blur/(2*r)) )) # angular displacement during exposure
speeds = theta_exposure_time / exposure_times                    # max rotation speed limited by blur

theta_readouts  = speeds * readout_time                   # angular displacement during readout
theta_per_frame = theta_exposure_time + theta_readouts    # total angular step per frame
times_per_frame = exposure_times + readout_time           # total time per frame

# ------------------------
# Total number of frames for a 180° rotation
# ------------------------
frames_per_180deg = 180 / theta_per_frame

# --- Additional code for visualization and angle list ---
# Choose exposure-time index to visualize (change as needed; keep within exposure_times range)
i_plot = -1  # for example; exposure_times[i_plot] is the exposure shown

exp_time = exposure_times[i_plot]
omega_max = speeds[i_plot]           # °/s (as you computed)
# angular motion while exposing (°). For your speeds this equals theta_max_deg
exposure_arc_deg = omega_max * exp_time
# angular motion during readout (°)
readout_arc_deg = omega_max * readout_time
# Angular step between triggers (°): exposure + readout
delta_theta = exposure_arc_deg + readout_arc_deg

# Number of frames that fit in 180 degrees
n_frames = int(np.floor(180.0 / delta_theta))
if n_frames < 1:
    raise ValueError("No frames fit in 180°. Reduce exposure or readout time or increase allowed blur.")

# Trigger angles (deg) where a trigger is sent and exposure begins
trigger_angles_deg = np.arange(0, n_frames) * delta_theta

# Convert degrees -> radians for polar plotting
trigger_angles_rad = np.deg2rad(trigger_angles_deg)
exposure_arc_rad = np.deg2rad(exposure_arc_deg)
readout_arc_rad = np.deg2rad(readout_arc_deg)

# --- Circular plot with legend inside the plot area ---
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'}, figsize=(7, 7))

# Plot triggers (blue dots)
ax.scatter(trigger_angles_rad, np.ones_like(trigger_angles_rad), s=40,
           label=f'Trigger (exposure starts)', zorder=3)

# Plot exposure arcs (solid)
for theta0 in trigger_angles_rad:
    arc_t = np.linspace(theta0, theta0 + exposure_arc_rad, 50)
    ax.plot(arc_t, np.ones_like(arc_t), lw=3, alpha=0.8,
            label='_nolegend_' if theta0 != trigger_angles_rad[0] else 'Exposure arc')

# Plot readout arcs (dashed)
for theta0 in trigger_angles_rad:
    start = theta0 + exposure_arc_rad
    arc_t = np.linspace(start, start + readout_arc_rad, 30)
    ax.plot(arc_t, np.ones_like(arc_t), lw=3, alpha=0.6, linestyle='--',
            label='_nolegend_' if theta0 != trigger_angles_rad[0] else 'Readout arc')

# Visual aids: 0° and 180° reference lines
ax.plot([0, 0], [0, 1.2], color='k', linewidth=0.6)
ax.plot([np.pi, np.pi], [0, 1.2], color='k', linewidth=0.6)

# Formatting
ax.set_rticks([])
ax.set_yticklabels([])
ax.set_ylim(0.9, 1.1)
ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)

# Title and legend inside the plot
title = (f"Exposure = {exp_time:.3f} s, Readout = {readout_time:.4f} s\n"
         f"Exposure arc = {exposure_arc_deg:.3f}°, Readout arc = {readout_arc_deg:.3f}°\n"
         f"Δθ/frame = {delta_theta:.3f}°, Frames in 180° = {n_frames}")
ax.set_title(title, va='bottom', pad=20)

# Move legend inside the plot
ax.legend(loc='upper right', bbox_to_anchor=(0.95, 0.95), frameon=True, framealpha=0.8)

# Add a box with input parameters
textstr = '\n'.join((
    r'Input parameters:',
    fr'Exposure = {exp_time:.3f} s',
    fr'Frame rate with 0s exposure time = {frame_rate_with_zero_exposure_time:.2f} fps',
    fr'Max blur = {set_blur} px'))
props = dict(boxstyle='round', facecolor='white', alpha=0.8)
ax.text(0.02, 0.02, textstr, transform=ax.figure.transFigure,
        fontsize=10, verticalalignment='bottom', bbox=props)

plt.show()
