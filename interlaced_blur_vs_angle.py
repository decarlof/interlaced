import numpy as np
import matplotlib.pyplot as plt

from interlaced_delta_angle import (
    compute_equally_spaced_multiturn_angles,
    compute_golden_angle_multiturn_angles,
    compute_corput_multiturn_angles,
    compute_timbir_multiturn_angles,
    compute_frame_time,
    _is_power_of_two,
)


def main():
    # Same parameters as interlaced_delta_angle.main()
    num_angles     = 10
    K_interlace    = 4
    rotation_start = 0.0
    rotation_stop  = 360.0
    exposure_time  = 0.1    # seconds
    size_x         = 2048   # detector horizontal size in pixels

    frame_time = compute_frame_time(exposure_time)

    # Compute angles for each mode (same order as main() datasets)
    angles_eq, _, _ = compute_equally_spaced_multiturn_angles(
        num_angles=num_angles, K_interlace=K_interlace,
        rotation_start=rotation_start, rotation_stop=rotation_stop,
    )
    angles_ga, _, _ = compute_golden_angle_multiturn_angles(
        num_angles=num_angles, K_interlace=K_interlace,
        rotation_start=rotation_start,
    )
    angles_vc, _, _ = compute_corput_multiturn_angles(
        num_angles=num_angles, K_interlace=K_interlace,
        rotation_start=rotation_start, rotation_stop=rotation_stop,
    )

    if _is_power_of_two(K_interlace):
        angles_tb, _, _ = compute_timbir_multiturn_angles(
            num_angles=num_angles, K_interlace=K_interlace,
            rotation_start=rotation_start,
        )
    else:
        angles_tb = None

    modes = [
        ("Equally Spaced",  angles_eq),
        ("Golden Angle",    angles_ga),
        ("Van der Corput",  angles_vc),
        ("TIMBIR",          angles_tb),
    ]

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharey=True)
    fig.suptitle(
        f"Projected motion blur vs. acquisition angle  —  "
        f"N={num_angles}, K={K_interlace}\n"
        f"exposure={exposure_time*1e3:.1f} ms, "
        f"frame time={frame_time*1e3:.3f} ms, "
        f"detector size_x={size_x} px\n"
        r"blur$(\theta)$ = $(size\_x/2)\cdot|\sin\theta|\cdot\omega\cdot t_{exp}$"
        "   [worst-case feature at detector edge]",
        fontsize=11,
    )

    # Sinusoidal envelope template for overlay (full acquisition range)
    total_angle = rotation_stop * K_interlace
    theta_env   = np.linspace(0, total_angle, 720 * K_interlace)
    sin_env     = np.abs(np.sin(np.radians(theta_env)))

    for ax, (title, angles_per_turn) in zip(axes.flat, modes):
        if angles_per_turn is None:
            ax.text(0.5, 0.5,
                    f"TIMBIR not available\n(K={K_interlace} is not a power of 2)",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=10, color="gray", style="italic")
            ax.set_title(title, fontsize=11, fontweight="bold")
            ax.set_xlabel("Acquisition angle (°)")
            ax.set_ylabel("Projected motion blur (px)")
            ax.grid(True, alpha=0.3)
            continue

        theta_acq = np.concatenate(angles_per_turn)
        delta     = np.diff(theta_acq)

        # Local velocity for each frame:
        #   frame 0 → no preceding gap, use min(delta) as conservative estimate
        #   frame i > 0 → velocity_i = delta[i-1] / frame_time
        preceding_gaps    = np.empty(len(theta_acq))
        preceding_gaps[0] = delta.min()
        preceding_gaps[1:]= delta

        velocities = preceding_gaps / frame_time   # °/s

        # Projected blur of a feature at radius r = size_x/2 from the rotation axis
        # whose projected velocity is maximum at θ = 90° and 270° (|sin θ| envelope):
        #   blur(θ) = r · dθ_rad · |sin(θ)|  =  (size_x/2) · ω·t_exp·π/180 · |sin(θ)|
        r = size_x / 2.0
        blur_proj = r * np.radians(velocities * exposure_time) * np.abs(np.sin(np.radians(theta_acq)))

        # One color per distinct velocity; dots and envelope curve share the same color
        vel_rounded   = np.round(velocities, decimals=6)
        unique_vels   = np.sort(np.unique(vel_rounded))
        vel_colors    = plt.cm.viridis(np.linspace(0.1, 0.9, len(unique_vels)))

        for vi, vel in enumerate(unique_vels):
            mask      = vel_rounded == vel
            c_vel     = vel_colors[vi]
            # Dots for this velocity group
            ax.scatter(theta_acq[mask], blur_proj[mask],
                       s=20, alpha=0.85, color=c_vel, zorder=3)
            # Sinusoidal envelope through those dots
            blur_env  = r * np.radians(vel * exposure_time) * sin_env
            ax.plot(theta_env, blur_env, color=c_vel, lw=1.5, alpha=0.75,
                    label=f"V={vel:.2f}°/s")

        ax.axhline(1.0, color="k", lw=0.8, ls=":", alpha=0.6, label="Nyquist (1 px)")

        ax.set_title(f"N={num_angles}, K={K_interlace}  —  {title}",
                     fontsize=11, fontweight="bold")
        ax.set_xlabel("Acquisition angle (°)")
        ax.set_ylabel("Projected motion blur (px)")
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
