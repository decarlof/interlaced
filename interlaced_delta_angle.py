import argparse
import numpy as np
import matplotlib.pyplot as plt


def _bit_reverse(val: int, bits: int) -> int:
    result = 0
    for _ in range(bits):
        result = (result << 1) | (val & 1)
        val >>= 1
    return result


def _ensure_power_of_two(K: int):
    if K < 1 or (K & (K - 1)) != 0:
        raise ValueError(f"K={K} must be a power of two.")


def _is_power_of_two(K: int) -> bool:
    return K >= 1 and (K & (K - 1)) == 0


def compute_equally_spaced_multiturn_angles(
    num_angles=180,
    K_interlace=3,
    rotation_start=0.0,
    rotation_stop=180.0,
    delta_theta=None,
    degrees=True,
):
    N = int(num_angles)
    K = int(K_interlace)

    if delta_theta is None:
        delta_theta = (rotation_stop - rotation_start) / N
    rotation_step = float(delta_theta)

    n = np.arange(N, dtype=float)
    angles_per_turn = []
    for k in range(K):
        theta_n = rotation_start + (n - k / K) * rotation_step + 360.0 * k
        angles_per_turn.append(theta_n)

    theta_interlaced = np.concatenate(angles_per_turn).astype(float)
    theta_monotonic = np.sort(theta_interlaced)

    return angles_per_turn, theta_interlaced, theta_monotonic


def compute_golden_angle_multiturn_angles(
    num_angles=180,
    K_interlace=3,
    rotation_start=0.0,
    degrees=True,
):
    N = int(num_angles)
    K = int(K_interlace)
    start_deg = float(rotation_start)

    if N <= 0 or K <= 0:
        raise ValueError("N and K must be > 0")

    golden_angle = 360.0 * (3.0 - np.sqrt(5.0)) / 2.0
    phi_inv = (np.sqrt(5.0) - 1.0) / 2.0

    base = np.array(
        [(start_deg + i * golden_angle) % 360.0 for i in range(N)],
        dtype=np.float64,
    )
    base.sort()

    angles_per_turn = []
    theta_list = []
    for k in range(K):
        if k == 0:
            block = base.copy()
        else:
            offset = (k / (N + 1.0)) * 360.0 * phi_inv
            block = np.sort((base + offset) % 360.0)

        unwrapped_block = start_deg + 360.0 * k + block
        angles_per_turn.append(unwrapped_block)
        theta_list.extend(unwrapped_block.tolist())

    theta_interlaced = np.asarray(theta_list, dtype=np.float64)
    theta_monotonic = np.sort(theta_interlaced)

    return angles_per_turn, theta_interlaced, theta_monotonic


def compute_corput_multiturn_angles(
    num_angles=180,
    K_interlace=4,
    rotation_start=0.0,
    rotation_stop=None,
    delta_theta=None,
    degrees=True,
):
    N = int(num_angles)
    K = int(K_interlace)
    start = float(rotation_start)

    if N <= 0 or K <= 0:
        raise ValueError("N and K must be > 0")

    if rotation_stop is None:
        rotation_stop = start + 360.0

    if delta_theta is not None:
        delta_theta = float(delta_theta)
    else:
        delta_theta = (float(rotation_stop) - start) / N

    base = start + np.arange(N, dtype=np.float64) * delta_theta

    bitsK = int(np.ceil(np.log2(K))) if K > 1 else 1
    MK = 1 << bitsK
    p_corput = np.array(
        [_bit_reverse(i, bitsK) for i in range(MK)], dtype=np.int64
    )
    p_corput = p_corput[p_corput < K]
    assert len(p_corput) == K
    offsets = (p_corput.astype(np.float64) / float(K)) * delta_theta

    bitsN = int(np.ceil(np.log2(N))) if N > 1 else 1
    MN = 1 << bitsN
    indices = np.array(
        [_bit_reverse(i, bitsN) for i in range(MN)], dtype=np.int64
    )
    indices = indices[indices < N]
    assert len(indices) == N

    angles_per_turn = []
    for k in range(K):
        offset = offsets[k]
        loop_angles = base[indices] + offset
        loop_angles_mod = np.mod(loop_angles - start, 360.0) + start
        loop_angles_mod = np.sort(loop_angles_mod)
        loop_angles_unwrapped = loop_angles_mod + 360.0 * k
        angles_per_turn.append(loop_angles_unwrapped)

    theta_unsorted = np.concatenate(angles_per_turn).astype(np.float64)
    theta_interlaced = np.sort(theta_unsorted)
    theta_monotonic = theta_interlaced.copy()

    return angles_per_turn, theta_interlaced, theta_monotonic


def compute_timbir_multiturn_angles(
    num_angles=180,
    K_interlace=4,
    rotation_start=0.0,
    degrees=True,
):
    N = int(num_angles)
    K = int(K_interlace)
    start_deg = float(rotation_start)

    _ensure_power_of_two(K)
    bits = int(np.log2(K))

    angles_per_turn = []
    for loop_turn in range(K):
        base_turn = 360.0 * loop_turn
        subloop = _bit_reverse(loop_turn, bits)
        turn_angles = []
        for i in range(N):
            idx = i * K + subloop
            angle_deg = idx * 360.0 / (N * K)
            turn_angles.append(start_deg + base_turn + angle_deg)
        angles_per_turn.append(np.asarray(turn_angles, dtype=np.float64))

    theta_interlaced = np.concatenate(angles_per_turn).astype(np.float64)
    theta_monotonic = np.sort(theta_interlaced)

    return angles_per_turn, theta_interlaced, theta_monotonic


def polar_plot_interlaced(
    angles_per_turn,
    theta_interlaced,
    theta_monotonic,
    title="Multi-turn acquisition angles (polar view)",
    degrees=True,
    show_monotonic=False,
    r0=1.0,
    dr=0.15,
    mono_offset=0.25,
    ax=None,
):
    K = len(angles_per_turn)
    N = len(angles_per_turn[0])

    if degrees:
        th_turns = [np.deg2rad(a % 360.0) for a in angles_per_turn]
        th_all = np.deg2rad(theta_interlaced % 360.0)
        th_mono = np.deg2rad(theta_monotonic % 360.0)
    else:
        th_turns = [a % (2 * np.pi) for a in angles_per_turn]
        th_all = theta_interlaced % (2 * np.pi)
        th_mono = theta_monotonic % (2 * np.pi)

    r_turns = [r0 + dr * k for k in range(K)]
    r_all = np.empty_like(th_all)
    for k in range(K):
        r_all[k * N : (k + 1) * N] = r_turns[k]

    standalone = ax is None
    if standalone:
        fig = plt.figure(figsize=(7, 7))
        ax = fig.add_subplot(111, projection="polar")

    for k in range(K):
        ax.scatter(
            th_turns[k],
            np.full(N, r_turns[k]),
            s=18,
            label=f"turn k={k}",
            alpha=0.9,
        )

    ax.plot(
        th_all, r_all, lw=1.0, alpha=0.35, color="k", label="acq order path"
    )

    if show_monotonic:
        ax.scatter(
            th_mono,
            np.full_like(th_mono, r0 - mono_offset),
            s=10,
            alpha=0.7,
            label="sorted (monotonic)",
        )

    ax.set_title(title, pad=20, fontsize=11)
    ax.set_rticks([])
    ax.set_ylim(r0 - mono_offset - 0.1, r0 + dr * (K - 1) + 0.2)
    ax.grid(True)
    ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.10), fontsize=7)

    if standalone:
        plt.tight_layout()
        plt.show()


def polar_plot_interlaced_grid(
    datasets,
    degrees=True,
    show_monotonic=False,
    r0=1.0,
    dr=0.15,
    mono_offset=0.25,
    suptitle="Comparison of Multi-Turn Acquisition Schemes",
):
    n_plots = len(datasets)
    ncols = min(n_plots, 2)
    nrows = int(np.ceil(n_plots / ncols))

    fig = plt.figure(figsize=(7 * ncols, 7 * nrows))
    fig.suptitle(suptitle, fontsize=15, y=1.02)

    for idx, ds in enumerate(datasets):
        ax = fig.add_subplot(nrows, ncols, idx + 1, projection="polar")
        if ds.get("unavailable"):
            ax.set_title(ds.get("title", ""), pad=20, fontsize=11)
            ax.text(0.5, 0.5, ds.get("message", "Not available"),
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=10, color="gray", style="italic")
            ax.set_rticks([])
            ax.grid(False)
            continue
        polar_plot_interlaced(
            ds["angles_per_turn"],
            ds["theta_interlaced"],
            ds["theta_monotonic"],
            title=ds["title"],
            degrees=degrees,
            show_monotonic=show_monotonic,
            r0=r0,
            dr=dr,
            mono_offset=mono_offset,
            ax=ax,
        )

    plt.tight_layout()
    plt.show()


def compute_delta_angles_acquisition_order(angles_per_turn):
    theta_acq = np.concatenate(angles_per_turn)
    delta_angles = np.diff(theta_acq)
    return delta_angles


def count_distinct_deltas(delta, decimals=6):
    return len(np.unique(np.round(delta, decimals=decimals)))


def plot_delta_angle_distributions(
    datasets,
    degrees=True,
    frame_time=None,
    exposure_time=None,
    size_x=None,
    suptitle=(
        r"$\Delta\theta$ between consecutive frames (acquisition order)"
        "\n— determines minimum detector exposure time at a given rotation speed —"
    ),
):
    n_plots = len(datasets)
    fig, axes = plt.subplots(
        3, n_plots, figsize=(5 * n_plots, 12),
        gridspec_kw={"height_ratios": [1, 1, 1]},
    )
    if n_plots == 1:
        axes = axes.reshape(-1, 1)

    fig.suptitle(suptitle, fontsize=13, y=1.03)

    colors = plt.cm.tab10.colors
    unit = "°" if degrees else "rad"

    all_deltas = []
    for ds in datasets:
        if ds.get("unavailable"):
            all_deltas.append(None)
        else:
            delta = compute_delta_angles_acquisition_order(ds["angles_per_turn"])
            all_deltas.append(delta)

    valid_deltas = [d for d in all_deltas if d is not None]
    global_min = min(d.min() for d in valid_deltas)
    global_max = max(d.max() for d in valid_deltas)
    margin = (global_max - global_min) * 0.1
    if margin < 1e-10:
        margin = 0.1
    shared_xlim = (global_min - margin, global_max + margin)
    ax_hist_axes = []

    for col, (ds, delta) in enumerate(zip(datasets, all_deltas)):
        if ds.get("unavailable"):
            msg = ds.get("message", "Not available")
            for row in range(3):
                ax = axes[row, col]
                ax.text(0.5, 0.5, msg, transform=ax.transAxes,
                        ha="center", va="center", fontsize=10,
                        color="gray", style="italic")
                ax.set_title(ds.get("title", ""), fontsize=11, fontweight="bold")
                ax.set_xticks([])
                ax.set_yticks([])
            continue

        angles_per_turn = ds["angles_per_turn"]
        K = len(angles_per_turn)
        N = len(angles_per_turn[0])

        has_negative = np.any(delta < 0)
        n_negative = np.sum(delta < 0)

        unique_vals = np.unique(np.round(delta, decimals=6))
        n_unique = len(unique_vals)

        # --- Top row: distribution view ---
        ax_hist = axes[0, col]

        if n_unique <= 20:
            counts = {}
            for v in np.round(delta, decimals=6):
                counts[v] = counts.get(v, 0) + 1
            vals = np.array(sorted(counts.keys()))
            cnts = np.array([counts[v] for v in vals])

            markerline, stemlines, baseline = ax_hist.stem(
                vals, cnts, linefmt="-", markerfmt="o", basefmt=" ",
            )
            markerline.set_color(colors[col % len(colors)])
            markerline.set_markersize(6)
            stemlines.set_color(colors[col % len(colors)])
            stemlines.set_linewidth(2)

            total_frames = N * K
            delta_rounded = np.round(delta, decimals=6)
            total_angle = float(angles_per_turn[-1][-1] - angles_per_turn[0][0])
            for v, c in zip(vals, cnts):
                collected = 1 + int(np.sum(delta_rounded >= v))
                eff = 100.0 * collected / total_frames
                label = f"{v:.3f}{unit}\n(n={c})\n{eff:.1f}%"
                if frame_time is not None:
                    vel = v / frame_time
                    t_scan = total_angle * frame_time / v
                    label += f"\nV={vel:.2f}°/s\nT={t_scan:.1f}s"
                    if exposure_time is not None and size_x is not None:
                        blur_px = size_x * np.sin(np.radians(vel * exposure_time) / 2)
                        label += f"\nB={blur_px:.2f}px"
                ax_hist.annotate(
                    label,
                    xy=(v, c), xytext=(0, 8),
                    textcoords="offset points",
                    ha="center", fontsize=6,
                )

            # Add headroom so annotations don't overlap with title
            max_count = cnts.max()
            ax_hist.set_ylim(0, max_count * 1.35)
        else:
            ax_hist.hist(
                delta,
                bins="auto",
                color=colors[col % len(colors)],
                edgecolor="k",
                alpha=0.75,
            )
            # Add headroom for histogram too
            y_top = ax_hist.get_ylim()[1]
            ax_hist.set_ylim(0, y_top * 1.15)

        title_str = ds["title"]
        title_str += f" ({n_unique} distinct values)"
        if has_negative:
            title_str += f"\n⚠ {n_negative} negative gaps"
        ax_hist.set_title(title_str, fontsize=11, fontweight="bold")
        ax_hist.set_xlabel(rf"$\Delta\theta$ ({unit})")
        ax_hist.set_ylabel("Count")
        ax_hist.set_xlim(shared_xlim)
        ax_hist.text(0.02, 0.97, f"N={N}, K={K}",
                     transform=ax_hist.transAxes, va="top", ha="left",
                     fontsize=8,
                     bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                               alpha=0.7, edgecolor="none"))
        ax_hist.grid(True, alpha=0.3)
        ax_hist_axes.append(ax_hist)

        # --- Middle row: delta vs acquisition index ---
        ax_stem = axes[1, col]
        base_color = colors[col % len(colors)]
        point_colors = [base_color if d >= 0 else "red" for d in delta]

        ax_stem.scatter(
            np.arange(len(delta)), delta,
            s=3, alpha=0.6, c=point_colors,
        )
        ax_stem.plot(
            np.arange(len(delta)), delta,
            lw=0.4, alpha=0.4, color=base_color,
        )
        ax_stem.axhline(
            np.mean(delta), color="r", ls="--", lw=1.2, alpha=0.8,
            label=f"mean = {np.mean(delta):.3f}{unit}",
        )
        if has_negative:
            ax_stem.axhline(0, color="k", ls="-", lw=0.8, alpha=0.5)
        for k in range(1, K):
            boundary = k * N - 1
            if boundary < len(delta):
                ax_stem.axvline(
                    boundary, color="gray", ls="--", lw=0.8, alpha=0.5,
                )
        ax_stem.set_xlabel("Acquisition index")
        ax_stem.set_ylabel(rf"$\Delta\theta$ ({unit})")
        ax_stem.set_title(r"$\Delta\theta$ vs. frame index", fontsize=10)
        ax_stem.legend(fontsize=7)
        ax_stem.grid(True, alpha=0.3)

        # --- Bottom row: per-turn breakdown ---
        ax_turn = axes[2, col]
        turn_colors = plt.cm.Set2.colors
        for k in range(K):
            turn_angles = angles_per_turn[k]
            delta_turn = np.diff(turn_angles)

            c = turn_colors[k % len(turn_colors)]
            ax_turn.plot(
                np.arange(len(delta_turn)),
                delta_turn,
                lw=0.8, alpha=0.8, color=c,
                label=f"turn {k} (mean={np.mean(delta_turn):.2f}{unit})",
            )
            ax_turn.scatter(
                np.arange(len(delta_turn)),
                delta_turn,
                s=6, alpha=0.6, color=c,
            )

        if has_negative:
            ax_turn.axhline(0, color="k", ls="-", lw=0.8, alpha=0.5)
        ax_turn.set_xlabel("Intra-turn frame index")
        ax_turn.set_ylabel(rf"$\Delta\theta$ ({unit})")
        ax_turn.set_title(r"$\Delta\theta$ per turn", fontsize=10)
        ax_turn.legend(fontsize=7)
        ax_turn.grid(True, alpha=0.3)

        fly_status = "✓ fly-scan OK" if not has_negative else f"✗ {n_negative} negative gaps"
        print(
            f"{ds['title']:>25s}:  "
            f"min={np.min(delta):8.3f}{unit}  "
            f"max={np.max(delta):8.3f}{unit}  "
            f"mean={np.mean(delta):8.3f}{unit}  "
            f"std={np.std(delta):8.3f}{unit}  "
            f"unique={n_unique:4d}  "
            f"[{fly_status}]"
        )

    if ax_hist_axes:
        shared_ylim = max(ax.get_ylim()[1] for ax in ax_hist_axes)
        for ax in ax_hist_axes:
            ax.set_ylim(0, shared_ylim)

    plt.tight_layout()
    plt.show()


def plot_distinct_deltas_vs_N_K(
    N_values=None,
    K_values=None,
    jitter=0,
):
    """
    Empirically measure the number of distinct delta_theta values
    as a function of N and K for all four schemes.

    Parameters
    ----------
    N_values : list of int or None
        Angles-per-turn values to test.
    K_values : list of int or None
        Number-of-turns values to test.
    jitter : float
        Vertical jitter to separate overlapping lines.
        0.0 = no jitter (lines overlap when equal),
        1.0 = default separation (±0.15).
    """
    if N_values is None:
        N_values = [10, 20, 50, 100, 200, 500, 1000]
    if K_values is None:
        K_values = [1, 2, 4, 8, 16]

    schemes = {
        "Equally Spaced": lambda N, K: compute_equally_spaced_multiturn_angles(
            num_angles=N, K_interlace=K, rotation_start=0.0, rotation_stop=360.0,
        ),
        "Golden Angle": lambda N, K: compute_golden_angle_multiturn_angles(
            num_angles=N, K_interlace=K, rotation_start=0.0,
        ),
        "Van der Corput": lambda N, K: compute_corput_multiturn_angles(
            num_angles=N, K_interlace=K, rotation_start=0.0, rotation_stop=360.0,
        ),
        "TIMBIR": lambda N, K: compute_timbir_multiturn_angles(
            num_angles=N, K_interlace=K, rotation_start=0.0,
        ),
    }

    markers = ["o", "s", "D", "^", "v", "P", "*", "X"]
    fig, axes = plt.subplots(1, len(schemes), figsize=(5 * len(schemes), 5))
    fig.suptitle(
        "Number of distinct $\\Delta\\theta$ values vs. N (angles per turn)\n"
        "for different K (number of turns)",
        fontsize=13, y=1.03,
    )

    colors_K = plt.cm.viridis(np.linspace(0.2, 0.9, len(K_values)))

    n_K = len(K_values)
    max_jitter = 0.15 * jitter
    jitter_offsets = np.linspace(-max_jitter, max_jitter, n_K)

    for ax, (scheme_name, scheme_fn) in zip(axes, schemes.items()):
        ax.set_title(scheme_name, fontsize=11, fontweight="bold")

        for ki, K in enumerate(K_values):
            n_distinct_list = []
            valid_N = []
            for N in N_values:
                try:
                    angles_per_turn, _, _ = scheme_fn(N, K)
                    delta = compute_delta_angles_acquisition_order(angles_per_turn)
                    n_dist = count_distinct_deltas(delta)
                    n_distinct_list.append(n_dist)
                    valid_N.append(N)
                except (ValueError, AssertionError):
                    continue

            if valid_N:
                jittered = [v + jitter_offsets[ki] for v in n_distinct_list]
                ax.plot(
                    valid_N, jittered,
                    linestyle="-", color=colors_K[ki], lw=1.5,
                    marker=markers[ki % len(markers)], ms=7,
                    label=f"K={K}",
                    alpha=0.85,
                )

        ax.set_xlabel("N (angles per turn)")
        ax.set_ylabel("# distinct $\\Delta\\theta$ values")
        ax.set_xscale("log")
        ax.yaxis.set_major_locator(plt.MaxNLocator(integer=True))
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Print table
    print(f"\n{'Scheme':>20s} {'K':>4s} {'N':>6s} {'#distinct':>10s}")
    print("-" * 45)
    for scheme_name, scheme_fn in schemes.items():
        for K in K_values:
            for N in N_values:
                try:
                    angles_per_turn, _, _ = scheme_fn(N, K)
                    delta = compute_delta_angles_acquisition_order(angles_per_turn)
                    n_dist = count_distinct_deltas(delta)
                    print(f"{scheme_name:>20s} {K:4d} {N:6d} {n_dist:10d}")
                except (ValueError, AssertionError):
                    pass

def plot_blur_vs_angle(
    datasets,
    frame_time,
    exposure_time,
    size_x=2048,
):
    N = len(datasets[0]["angles_per_turn"][0]) if not datasets[0].get("unavailable") else "?"
    K = len(datasets[0]["angles_per_turn"])    if not datasets[0].get("unavailable") else "?"

    n_plots = len(datasets)
    ncols = min(n_plots, 2)
    nrows = int(np.ceil(n_plots / ncols))

    fig, axes = plt.subplots(nrows, ncols, figsize=(7 * ncols, 5 * nrows), sharey=True)
    if nrows == 1 and ncols == 1:
        axes = np.array([[axes]])
    elif nrows == 1 or ncols == 1:
        axes = np.array(axes).reshape(nrows, ncols)

    fig.suptitle(
        f"Projected motion blur vs. acquisition angle  —  N={N}, K={K}\n"
        f"exposure={exposure_time*1e3:.1f} ms, "
        f"frame time={frame_time*1e3:.3f} ms, "
        f"detector size_x={size_x} px\n"
        r"blur$(\theta)$ = $(size\_x/2)\cdot|\sin\theta|\cdot\omega\cdot t_{exp}$"
        "   [worst-case feature at detector edge]",
        fontsize=11,
    )

    r = size_x / 2.0

    for idx, (ax, ds) in enumerate(zip(axes.flat, datasets)):
        if ds.get("unavailable"):
            ax.text(0.5, 0.5, ds.get("message", "Not available"),
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=10, color="gray", style="italic")
            ax.set_title(ds.get("title", ""), fontsize=11, fontweight="bold")
            ax.set_xlabel("Acquisition angle (°)")
            ax.set_ylabel("Projected motion blur (px)")
            ax.grid(True, alpha=0.3)
            continue

        angles_per_turn = ds["angles_per_turn"]
        theta_acq = np.concatenate(angles_per_turn)
        delta     = np.diff(theta_acq)

        preceding_gaps     = np.empty(len(theta_acq))
        preceding_gaps[0]  = delta.min()
        preceding_gaps[1:] = delta
        velocities = preceding_gaps / frame_time   # °/s

        blur_proj = r * np.radians(velocities * exposure_time) * np.abs(np.sin(np.radians(theta_acq)))

        # Sinusoidal envelope over the full acquisition range
        total_angle = float(angles_per_turn[-1][-1] - angles_per_turn[0][0])
        theta_env   = np.linspace(0, total_angle, 2000)
        sin_env     = np.abs(np.sin(np.radians(theta_env)))

        # One color per distinct velocity
        vel_rounded = np.round(velocities, decimals=6)
        unique_vels = np.sort(np.unique(vel_rounded))
        vel_colors  = plt.cm.viridis(np.linspace(0.1, 0.9, len(unique_vels)))

        for vi, vel in enumerate(unique_vels):
            mask  = vel_rounded == vel
            c_vel = vel_colors[vi]
            ax.scatter(theta_acq[mask], blur_proj[mask],
                       s=20, alpha=0.85, color=c_vel, zorder=3)
            ax.plot(theta_env, r * np.radians(vel * exposure_time) * sin_env,
                    color=c_vel, lw=1.5, alpha=0.75, label=f"V={vel:.2f}°/s")

        ax.axhline(1.0, color="k", lw=0.8, ls=":", alpha=0.6, label="Nyquist (1 px)")
        ax.set_title(ds["title"], fontsize=11, fontweight="bold")
        ax.set_xlabel("Acquisition angle (°)")
        ax.set_ylabel("Projected motion blur (px)")
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def plot_equally_spaced_comparison(
    num_angles=10,
    K_interlace=4,
    rotation_start=0.0,
    rotation_stop=360.0,
    frame_time=None,
    exposure_time=0.1,
    size_x=2048,
    degrees=True,
    dr=0.25,
):
    """Compare the old (+k/K offset) and new (-k/K offset) Equally Spaced formulas.

    Shows a 2×2 page:
      top row    — polar acquisition plots (old | new)
      bottom row — Δθ count distributions  (old | new)
    """
    N = int(num_angles)
    K = int(K_interlace)
    rotation_step = (rotation_stop - rotation_start) / N
    n_arr = np.arange(N, dtype=float)
    unit  = "°" if degrees else "rad"

    # OLD formula: first frame of turn k is k/K * step ABOVE k×360°
    angles_old = [rotation_start + (n_arr + k / K) * rotation_step + 360.0 * k
                  for k in range(K)]
    # NEW formula: first frame of turn k is k/K * step BELOW k×360°
    angles_new = [rotation_start + (n_arr - k / K) * rotation_step + 360.0 * k
                  for k in range(K)]

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(
        f"Equally Spaced interlaced — offset comparison  (N={N}, K={K}, step={rotation_step:.3f}°)\n"
        f"Left: first frame ABOVE k×360°  (old  +k/K·step) | "
        f"Right: first frame BELOW k×360°  (new  −k/K·step)",
        fontsize=11,
    )

    ax_pol_old = fig.add_subplot(2, 2, 1, projection="polar")
    ax_pol_new = fig.add_subplot(2, 2, 2, projection="polar")
    ax_cnt_old = fig.add_subplot(2, 2, 3)
    ax_cnt_new = fig.add_subplot(2, 2, 4)

    colors = plt.cm.tab10.colors

    # --- polar plots (top row) ---
    for ax_pol, angles_per_turn, label in [
        (ax_pol_old, angles_old, "old: +k/K·step"),
        (ax_pol_new, angles_new, "new: −k/K·step"),
    ]:
        theta_mono = np.sort(np.concatenate(angles_per_turn))
        polar_plot_interlaced(
            angles_per_turn,
            np.concatenate(angles_per_turn),
            theta_mono,
            title=label,
            degrees=degrees,
            dr=dr,
            ax=ax_pol,
        )

    # --- count distribution plots (bottom row) ---
    # Determine shared x-axis limits across both datasets
    all_deltas = [
        compute_delta_angles_acquisition_order(angles_old),
        compute_delta_angles_acquisition_order(angles_new),
    ]
    global_min = min(d.min() for d in all_deltas)
    global_max = max(d.max() for d in all_deltas)
    margin = (global_max - global_min) * 0.15 or 0.5
    shared_xlim = (global_min - margin, global_max + margin)
    shared_ylim = None

    ax_cnt_list = []
    for ax_cnt, angles_per_turn, delta, label, ci in [
        (ax_cnt_old, angles_old, all_deltas[0], "old: +k/K·step", 0),
        (ax_cnt_new, angles_new, all_deltas[1], "new: −k/K·step", 1),
    ]:
        delta_rounded = np.round(delta, decimals=6)
        unique_vals   = np.sort(np.unique(delta_rounded))
        counts        = {v: int(np.sum(delta_rounded == v)) for v in unique_vals}
        vals          = np.array(sorted(counts.keys()))
        cnts          = np.array([counts[v] for v in vals])

        total_frames = N * K
        total_angle  = float(angles_per_turn[-1][-1] - angles_per_turn[0][0])

        c = colors[ci % len(colors)]
        ml, sl, _ = ax_cnt.stem(vals, cnts, linefmt="-", markerfmt="o", basefmt=" ")
        ml.set_color(c); ml.set_markersize(6)
        sl.set_color(c); sl.set_linewidth(2)

        for v, cnt in zip(vals, cnts):
            collected = 1 + int(np.sum(delta_rounded >= v))
            eff       = 100.0 * collected / total_frames
            lbl       = f"{v:.3f}{unit}\n(n={cnt})\n{eff:.1f}%"
            if frame_time is not None:
                vel    = v / frame_time
                t_scan = total_angle * frame_time / v
                lbl   += f"\nV={vel:.2f}°/s\nT={t_scan:.1f}s"
                if size_x is not None:
                    blur_px = size_x * np.sin(np.radians(vel * exposure_time) / 2)
                    lbl    += f"\nB={blur_px:.2f}px"
            ax_cnt.annotate(lbl, xy=(v, cnt), xytext=(0, 8),
                            textcoords="offset points", ha="center", fontsize=7)

        ax_cnt.set_title(f"Δθ distribution — {label}", fontsize=10, fontweight="bold")
        ax_cnt.set_xlabel(rf"$\Delta\theta$ ({unit})")
        ax_cnt.set_ylabel("Count")
        ax_cnt.set_xlim(shared_xlim)
        ax_cnt.set_ylim(0, cnts.max() * 1.5)
        ax_cnt.text(0.02, 0.97, f"N={N}, K={K}",
                    transform=ax_cnt.transAxes, va="top", ha="left", fontsize=8,
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white",
                              alpha=0.7, edgecolor="none"))
        ax_cnt.grid(True, alpha=0.3)
        ax_cnt_list.append(ax_cnt)

    # Unify y-axis across both count subplots
    shared_ylim = max(ax.get_ylim()[1] for ax in ax_cnt_list)
    for ax in ax_cnt_list:
        ax.set_ylim(0, shared_ylim)

    plt.tight_layout()
    plt.show()


def compute_frame_time(
    exposure_time,
    camera_model="Grasshopper3 GS3-U3-23S6M",
    pixel_format="Mono8",
    video_mode="Mode0",
):
    """Compute the minimum time between camera triggers (frame time).

    Returns the frame time in seconds, which is the larger of:
    - ``exposure_time`` plus a camera-specific readout margin, or
    - the camera readout time plus 1 ms.

    Readout times are empirical values measured at 100 µs exposure with
    1000 frames without dropping.  The ``video_mode`` parameter is only
    relevant for the Grasshopper3 GS3-U3-23S6M camera.

    Parameters
    ----------
    exposure_time : float
        Requested camera exposure time in seconds.
    camera_model : str
        Camera model string as reported by the detector driver.
    pixel_format : str
        Pixel format string (e.g. ``'Mono8'``, ``'Mono16'``).
    video_mode : str
        Video mode string; only used for the GS3-U3-23S6M model.

    Returns
    -------
    float
        Frame time in seconds, or 0 if the camera/format combination is
        not recognised.
    """
    readout = None
    readout_margin = 1.01

    if camera_model == "Grasshopper3 GS3-U3-23S6M":
        readout_times = {
            "Mono8":        {"Mode0": 6.2,  "Mode1": 6.2, "Mode5": 6.2, "Mode7": 7.9},
            "Mono12Packed": {"Mode0": 9.2,  "Mode1": 6.2, "Mode5": 6.2, "Mode7": 11.5},
            "Mono16":       {"Mode0": 12.2, "Mode1": 6.2, "Mode5": 6.2, "Mode7": 12.2},
        }
        readout = readout_times[pixel_format][video_mode] / 1000.0

    elif camera_model == "Grasshopper3 GS3-U3-51S5M":
        readout_times = {
            "Mono8": 6.18, "Mono12Packed": 8.20, "Mono12p": 8.20, "Mono16": 12.34,
        }
        readout = readout_times[pixel_format] / 1000.0

    elif camera_model == "Oryx ORX-10G-51S5M":
        readout_margin = 1.05
        readout_times = {"Mono8": 6.18, "Mono12Packed": 8.20, "Mono16": 12.34}
        readout = readout_times[pixel_format] / 1000.0

    elif camera_model == "Oryx ORX-10G-310S9M":
        readout_margin = 1.20
        readout_times = {"Mono8": 30.0, "Mono12Packed": 30.0, "Mono16": 30.0}
        readout = readout_times[pixel_format] / 1000.0

    elif camera_model == "Q-12A180-Fm/CXP-6":
        readout_times = {"Mono8": 5.35}
        readout = readout_times[pixel_format] / 1000.0

    elif camera_model == "Blackfly S BFS-PGE-161S7M":
        readout_margin = 1.035
        readout_times = {"Mono8": 83.4, "Mono12Packed": 100.0, "Mono16": 142.86}
        readout = readout_times[pixel_format] / 1000.0

    if readout is None:
        print(
            f"[compute_frame_time] Unsupported camera/format/mode: "
            f"{camera_model!r} / {pixel_format!r} / {video_mode!r}"
        )
        return 0

    frame_time = exposure_time * readout_margin
    if frame_time < readout:
        frame_time = readout + 0.001
    return frame_time


def pv_callback_efficiency(
    InterlacedRotationStart=0.0,
    InterlacedNumAngles=180,
    InterlacedNumberOfRotation=4,
    InterlacedMode=0,
    exposure_time=0.001,
    camera_model="Grasshopper3 GS3-U3-23S6M",
    pixel_format="Mono8",
    video_mode="Mode0",
    rotation_stop=360.0,
    frame_time_override=None,
    InterlacedEfficiencyRequested=100,
    size_x=2048,
):
    """Compute and report acquisition efficiency for the selected interlaced mode.

    Mirrors the role of a PV callback: when any scan parameter changes this
    function recomputes and prints:

    * **InterlacedPSOWindowStep** = ``(rotation_stop - rotation_start) / N``
      (identical to the real tomoScan formula; written to the MEDM display).

    * **InterlacedScanTime** = total scan duration at the *slowest* stage
      velocity (minimum Δθ), i.e. all frames are captured.

    * An efficiency table with one row per distinct Δθ value of the chosen
      mode.  Each row shows what happens when the rotation stage runs at
      ``velocity = Δθ / frame_time``:

        - **Collected** – frames whose preceding angular gap ≥ chosen Δθ
          (camera has enough time to read out).  Frame 0 is always collected.
        - **Dropped**   – frames whose preceding gap < chosen Δθ.
        - **Efficiency** – collected / total_frames × 100 %.

    Parameters
    ----------
    InterlacedRotationStart : float
        Start angle in degrees.
    InterlacedNumAngles : int
        Projections per rotation (N).
    InterlacedNumberOfRotation : int
        Number of rotations / interlace factor (K).
    InterlacedMode : int
        0 = Uniform, 1 = TIMBIR, 2 = Golden Angle, 3 = Van der Corput.
    exposure_time : float
        Camera exposure time in seconds.
    camera_model : str
        Camera model string (see ``compute_frame_time``).
    pixel_format : str
        Camera pixel format string.
    video_mode : str
        Camera video mode string (GS3-U3-23S6M only).
    rotation_stop : float
        End angle in degrees.

    InterlacedEfficiencyRequested : int
        Desired acquisition efficiency (0–100 %).  The last row whose
        efficiency meets or exceeds this value is marked with ``X`` in the
        printed table and returned as ``selected_row``.

    Returns
    -------
    dict or None
        ``{"delta_theta", "velocity", "scan_time", "collected", "dropped", "efficiency"}``
        for the selected row, or ``None`` if no row meets the requested efficiency
        or the camera/mode is not supported.
    """
    N = int(InterlacedNumAngles)
    K = int(InterlacedNumberOfRotation)
    start = float(InterlacedRotationStart)
    stop  = float(rotation_stop)
    total_frames = N * K

    mode_names = {0: "Uniform", 1: "TIMBIR", 2: "Golden Angle", 3: "Van der Corput"}

    if frame_time_override is not None:
        frame_time = float(frame_time_override)
    else:
        frame_time = compute_frame_time(exposure_time, camera_model, pixel_format, video_mode)
    if frame_time == 0:
        print("[pv_callback_efficiency] frame_time = 0: unsupported camera/format.")
        return {}

    # InterlacedPSOWindowStep — same formula as tomoScan (360 / N, or scaled range)
    pso_step = (stop - start) / N

    # --- compute angles for the selected mode ---
    try:
        if InterlacedMode == 0:
            angles_per_turn, _, _ = compute_equally_spaced_multiturn_angles(
                num_angles=N, K_interlace=K, rotation_start=start, rotation_stop=stop)
        elif InterlacedMode == 1:
            if not _is_power_of_two(K):
                raise ValueError(f"TIMBIR requires K to be a power of 2 (got K={K})")
            angles_per_turn, _, _ = compute_timbir_multiturn_angles(
                num_angles=N, K_interlace=K, rotation_start=start)
        elif InterlacedMode == 2:
            angles_per_turn, _, _ = compute_golden_angle_multiturn_angles(
                num_angles=N, K_interlace=K, rotation_start=start)
        elif InterlacedMode == 3:
            angles_per_turn, _, _ = compute_corput_multiturn_angles(
                num_angles=N, K_interlace=K, rotation_start=start, rotation_stop=stop)
        else:
            raise ValueError(f"Unknown InterlacedMode={InterlacedMode}")
    except (ValueError, AssertionError) as exc:
        print(f"[pv_callback_efficiency] Mode {InterlacedMode} not available: {exc}")
        return {}

    delta         = compute_delta_angles_acquisition_order(angles_per_turn)
    delta_rounded = np.round(delta, decimals=6)
    unique_dt     = np.sort(np.unique(delta_rounded))
    total_angle   = float(angles_per_turn[-1][-1] - angles_per_turn[0][0])

    # InterlacedScanTime — duration at minimum (slowest) velocity
    scan_time_min = total_angle * frame_time / float(unique_dt[0])

    # --- build efficiency rows ---
    rows = []
    for dt in unique_dt:
        vel       = dt / frame_time
        t_scan    = total_angle * frame_time / dt
        collected = 1 + int(np.sum(delta_rounded >= dt))
        dropped   = total_frames - collected
        eff       = 100.0 * collected / total_frames
        blur_px   = size_x * np.sin(np.radians(vel * exposure_time) / 2)
        rows.append(dict(delta_theta=dt, velocity=vel, scan_time=t_scan,
                         collected=collected, dropped=dropped, efficiency=eff,
                         blur_px=blur_px))

    # Last row whose efficiency meets or exceeds the requested value
    req = float(InterlacedEfficiencyRequested)
    selected_idx = None
    for i, row in enumerate(rows):
        if row['efficiency'] >= req:
            selected_idx = i

    # --- print report ---
    W = 94
    print(f"\n{'='*W}")
    print(f"  Interlaced Scan Efficiency  —  Mode {InterlacedMode}: {mode_names[InterlacedMode]}")
    print(f"  N={N} angles/turn | K={K} turns | {start:.1f}° → {stop:.1f}°")
    ft_src = "override" if frame_time_override is not None else f"exposure {exposure_time*1e3:.3f} ms"
    print(f"  Frame time: {frame_time*1e3:.3f} ms  ({ft_src})")
    print(f"  InterlacedPSOWindowStep: {pso_step:.4f}°  |  Total frames: {total_frames}")
    print(f"  InterlacedScanTime (min vel): {scan_time_min:.2f} s  |  {len(unique_dt)} distinct Δθ values")
    print(f"  Efficiency requested: {InterlacedEfficiencyRequested}%  |  Image size_x: {size_x} px")
    print(f"{'='*W}")
    print()
    print(f"  At velocity Vᵢ = Δθᵢ / frame_time the stage moves Δθᵢ per frame.")
    print(f"  Frames preceded by a gap < Δθᵢ cannot be read out in time and are DROPPED.")
    print()

    cols = (4, 12, 15, 14, 11, 9, 11, 10)
    header = (
        f"  {'#':>{cols[0]}}    {'Δθ (°)':>{cols[1]}}  {'Vel (°/s)':>{cols[2]}}"
        f"  {'Scan time (s)':>{cols[3]}}  {'Collected':>{cols[4]}}"
        f"  {'Dropped':>{cols[5]}}  {'Efficiency':>{cols[6]}}  {'Blur (px)':>{cols[7]}}"
    )
    sep_len = sum(cols) + 2 * len(cols) + 3
    print(header)
    print(f"  {'-' * sep_len}")

    for i, row in enumerate(rows):
        marker = "X" if i == selected_idx else " "
        print(
            f"  {i:>{cols[0]}}  {marker}  {row['delta_theta']:>{cols[1]}.6f}"
            f"  {row['velocity']:>{cols[2]}.4f}  {row['scan_time']:>{cols[3]}.2f}"
            f"  {row['collected']:>{cols[4]}d}  {row['dropped']:>{cols[5]}d}"
            f"  {row['efficiency']:>{cols[6]-1}.1f}%  {row['blur_px']:>{cols[7]}.2f}"
        )

    if selected_idx is not None:
        sr = rows[selected_idx]
        print(
            f"\n  [X] Row {selected_idx} selected: "
            f"Δθ={sr['delta_theta']:.6f}°  Vel={sr['velocity']:.4f}°/s  "
            f"Eff={sr['efficiency']:.1f}%  Scan time={sr['scan_time']:.2f} s  Blur={sr['blur_px']:.2f} px"
        )
    else:
        print(f"\n  No row meets the requested efficiency of {InterlacedEfficiencyRequested}%")

    print(f"\n{'='*W}\n")
    if selected_idx is None:
        return None
    return rows[selected_idx]


def main(plot=False, efficiency=100):
    num_angles = 10
    K_interlace = 4
    rotation_start = 0.0
    rotation_stop = 360.0
    exposure_time = 0.1          # seconds
    size_x        = 2048         # detector horizontal size in pixels
    degrees = True
    show_monotonic = True
    dr = 0.25

    angles_eq, theta_eq, theta_eq_mono = compute_equally_spaced_multiturn_angles(
        num_angles=num_angles,
        K_interlace=K_interlace,
        rotation_start=rotation_start,
        rotation_stop=rotation_stop,
        degrees=degrees,
    )

    angles_ga, theta_ga, theta_ga_mono = compute_golden_angle_multiturn_angles(
        num_angles=num_angles,
        K_interlace=K_interlace,
        rotation_start=rotation_start,
        degrees=degrees,
    )

    angles_vc, theta_vc, theta_vc_mono = compute_corput_multiturn_angles(
        num_angles=num_angles,
        K_interlace=K_interlace,
        rotation_start=rotation_start,
        rotation_stop=rotation_stop,
        degrees=degrees,
    )
    if _is_power_of_two(K_interlace):
        angles_tb, theta_tb, theta_tb_mono = compute_timbir_multiturn_angles(
            num_angles=num_angles,
            K_interlace=K_interlace,
            rotation_start=rotation_start,
            degrees=degrees,
        )
        timbir_dataset = {
            "angles_per_turn": angles_tb,
            "theta_interlaced": theta_tb,
            "theta_monotonic": theta_tb_mono,
            "title": "TIMBIR",
        }
    else:
        timbir_dataset = {
            "title": "TIMBIR",
            "unavailable": True,
            "message": f"TIMBIR only works with a\npower-of-2 number of turns\n(K = 1, 2, 4, 8, …)\nGot K={K_interlace}.",
        }

    datasets = [
        {
            "angles_per_turn": angles_eq,
            "theta_interlaced": theta_eq,
            "theta_monotonic": theta_eq_mono,
            "title": "Equally Spaced",
        },
        {
            "angles_per_turn": angles_ga,
            "theta_interlaced": theta_ga,
            "theta_monotonic": theta_ga_mono,
            "title": "Golden Angle",
        },
        {
            "angles_per_turn": angles_vc,
            "theta_interlaced": theta_vc,
            "theta_monotonic": theta_vc_mono,
            "title": "Van der Corput",
        },
        timbir_dataset,
    ]

    if plot:
        plot_equally_spaced_comparison(
            num_angles=num_angles,
            K_interlace=K_interlace,
            rotation_start=rotation_start,
            rotation_stop=rotation_stop,
            frame_time=compute_frame_time(exposure_time),
            exposure_time=exposure_time,
            size_x=size_x,
            degrees=degrees,
            dr=dr,
        )

        polar_plot_interlaced_grid(
            datasets,
            degrees=degrees,
            show_monotonic=show_monotonic,
            dr=dr,
        )

        plot_delta_angle_distributions(
            datasets,
            degrees=degrees,
            frame_time=compute_frame_time(exposure_time),
            exposure_time=exposure_time,
            size_x=size_x,
        )

        plot_distinct_deltas_vs_N_K()

        plot_blur_vs_angle(
            datasets,
            frame_time=compute_frame_time(exposure_time),
            exposure_time=exposure_time,
            size_x=size_x,
        )

    for mode in range(4):
        pv_callback_efficiency(
            InterlacedRotationStart=rotation_start,
            InterlacedNumAngles=num_angles,
            InterlacedNumberOfRotation=K_interlace,
            InterlacedMode=mode,
            rotation_stop=rotation_stop,
            exposure_time=exposure_time,
            InterlacedEfficiencyRequested=efficiency,
            size_x=size_x,
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Interlaced scan angle analysis")
    parser.add_argument("--plot", action="store_true", help="Show matplotlib plots")
    args = parser.parse_args()
    main(plot=args.plot)
