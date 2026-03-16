"""Print interlaced scan efficiency tables for all modes and multiple parameter sets.

No EPICS required.  Uses the same angle-generation logic as tomoscan_fpga_pso.py.

Usage:
    python interlaced_preview.py
"""
import numpy as np


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _bit_reverse(n: int, bits: int) -> int:
    if bits == 0:
        return 0
    return int(f"{n:0{bits}b}"[::-1], 2)


# ---------------------------------------------------------------------------
# Angle generators  (ported from TomoScanFPGAPSO)
# ---------------------------------------------------------------------------

def angles_uniform(N, K, start_deg=0.0):
    """Uniform multi-turn: N angles/turn, K turns, 360/N spacing, k/K sub-offset."""
    delta_theta = 360.0 / N
    n = np.arange(N, dtype=np.float64)
    blocks = [start_deg + (n + k / K) * delta_theta + 360.0 * k for k in range(K)]
    return np.concatenate(blocks)


def angles_timbir(N, K, start_deg=0.0):
    """TIMBIR: bit-reversed rotation order; K must be a power of 2."""
    if K <= 0 or (K & (K - 1)) != 0:
        raise ValueError(f"TIMBIR requires K to be a power of 2 (got K={K})")
    bits = int(np.log2(K))
    theta = []
    for loop_turn in range(K):
        base_turn = 360.0 * loop_turn
        subloop = _bit_reverse(loop_turn, bits)
        for i in range(N):
            idx = i * K + subloop
            angle_deg = idx * 360.0 / (N * K)
            theta.append(start_deg + base_turn + angle_deg)
    return np.asarray(theta, dtype=np.float64)


def angles_goldenangle(N, K, start_deg=0.0):
    """Golden-angle interlaced: sorted blocks per turn, shifted by phi^-1 offset."""
    golden = 360.0 * (3.0 - np.sqrt(5.0)) / 2.0
    phi_inv = (np.sqrt(5.0) - 1.0) / 2.0
    base = np.sort(np.array([(start_deg + i * golden) % 360.0 for i in range(N)]))
    theta = []
    for k in range(K):
        if k == 0:
            block = base
        else:
            offset = (k / (N + 1.0)) * 360.0 * phi_inv
            block = np.sort((base + offset) % 360.0)
        theta.extend((start_deg + 360.0 * k + block).tolist())
    return np.asarray(theta, dtype=np.float64)


def angles_corput(N, K, start_deg=0.0):
    """Van der Corput: bit-reversed intra- and inter-rotation order, then sorted."""
    delta_theta = 360.0 / N

    bitsK = int(np.ceil(np.log2(K))) if K > 1 else 1
    MK = 1 << bitsK
    p_corput = np.array([_bit_reverse(i, bitsK) for i in range(MK)], dtype=np.int64)
    p_corput = p_corput[p_corput < K]

    offsets = (p_corput.astype(np.float64) / K) * delta_theta

    bitsN = int(np.ceil(np.log2(N))) if N > 1 else 1
    MN = 1 << bitsN
    indices = np.array([_bit_reverse(i, bitsN) for i in range(MN)], dtype=np.int64)
    indices = indices[indices < N]

    base = start_deg + np.arange(N, dtype=np.float64) * delta_theta

    blocks = []
    for k in range(K):
        loop_angles = base[indices] + offsets[k]
        loop_mod = np.mod(loop_angles - start_deg, 360.0) + start_deg
        blocks.append(loop_mod + 360.0 * k)

    theta = np.sort(np.concatenate(blocks))
    return theta


ANGLE_FUNCS = {
    0: ("Uniform",         angles_uniform),
    1: ("TIMBIR",          angles_timbir),
    2: ("Golden Angle",    angles_goldenangle),
    3: ("Van der Corput",  angles_corput),
}


# ---------------------------------------------------------------------------
# Efficiency table
# ---------------------------------------------------------------------------

def efficiency_table(N, K, mode, start_deg, exposure_time, size_x,
                     readout_margin=1.01):
    """Return list of rows with efficiency data, or raise on error."""
    name, fn = ANGLE_FUNCS[mode]
    flat = fn(N, K, start_deg)

    frame_time = exposure_time * readout_margin
    total_frames = N * K
    total_angle = float(flat[-1] - flat[0])

    delta         = np.diff(flat)
    delta_rounded = np.round(delta, decimals=6)
    unique_dt     = np.sort(np.unique(delta_rounded))

    if len(unique_dt) == 0 or unique_dt[0] <= 0:
        raise ValueError("No positive Δθ values")

    rows = []
    for dt in unique_dt:
        vel       = dt / frame_time
        t_scan    = total_angle / vel
        collected = 1 + int(np.sum(delta_rounded >= dt))
        eff       = 100.0 * collected / total_frames
        blur      = size_x * np.sin(np.radians(vel * exposure_time) / 2)
        rows.append(dict(dt=dt, velocity=vel, scan_time=t_scan,
                         collected=collected, dropped=total_frames - collected,
                         efficiency=eff, blur_px=blur))
    return rows


def select_row(rows, req_pct):
    """Return last row whose efficiency >= req_pct, or None."""
    selected = None
    for row in rows:
        if row['efficiency'] >= req_pct:
            selected = row
    return selected


# ---------------------------------------------------------------------------
# Printing
# ---------------------------------------------------------------------------

MODE_NAMES = {0: "Uniform", 1: "TIMBIR", 2: "Golden Angle", 3: "Van der Corput"}

SEP = "=" * 96

def print_table(N, K, mode, start_deg, stop_deg, exposure_time, size_x,
                req_pct, readout_margin=1.01):
    frame_time = exposure_time * readout_margin
    name = MODE_NAMES[mode]
    total_frames = N * K

    print(SEP)
    print(f"  Interlaced Scan Efficiency  —  Mode {mode}: {name}")
    print(f"  N={N} angles/turn | K={K} turns | {start_deg:.1f}° → {stop_deg:.1f}°")
    print(f"  Frame time: {frame_time*1000:.3f} ms  (exposure {exposure_time*1000:.3f} ms)")
    print(f"  Total frames: {total_frames}  |  size_x: {size_x} px  |  req: {req_pct:.0f}%")
    print(SEP)

    try:
        rows = efficiency_table(N, K, mode, start_deg, exposure_time, size_x, readout_margin)
    except Exception as e:
        print(f"  ERROR: {e}")
        print(SEP)
        return

    sel = select_row(rows, req_pct)

    n_distinct = len(rows)
    min_vel_row = rows[0]
    print(f"  {n_distinct} distinct Δθ values  |  "
          f"InterlacedScanTime (min vel): {min_vel_row['scan_time']:.2f} s")
    print()
    hdr = f"  {'#':>4}  {'Δθ (°)':>14}  {'Vel (°/s)':>12}  {'Scan time (s)':>13}  "
    hdr += f"{'Collected':>10}  {'Dropped':>8}  {'Efficiency':>11}  {'Blur (px)':>10}"
    print(hdr)
    print("  " + "-" * 92)
    for i, row in enumerate(rows):
        mark = " X" if row is sel else "  "
        print(f"  {i:>2}{mark}  {row['dt']:>14.6f}  {row['velocity']:>12.4f}  "
              f"{row['scan_time']:>13.2f}  {row['collected']:>10d}  {row['dropped']:>8d}  "
              f"{row['efficiency']:>10.1f}%  {row['blur_px']:>10.2f}")
    print()
    if sel:
        print(f"  [X] Row selected: Δθ={sel['dt']:.6f}°  Vel={sel['velocity']:.4f}°/s  "
              f"Eff={sel['efficiency']:.1f}%  Scan time={sel['scan_time']:.2f} s  "
              f"Blur={sel['blur_px']:.2f} px")
    else:
        print(f"  [!] No row meets efficiency >= {req_pct:.0f}%")
    print(SEP)
    print()


# ---------------------------------------------------------------------------
# Parameter sets and main
# ---------------------------------------------------------------------------

PARAM_SETS = [
    dict(N=10,  K=4,  start_deg=0.0, stop_deg=1440.0, exposure_time=0.1, size_x=3232, req_pct=100.0),
    dict(N=100, K=4,  start_deg=0.0, stop_deg=1440.0, exposure_time=0.1, size_x=3232, req_pct=100.0),
    dict(N=100, K=8,  start_deg=0.0, stop_deg=1440.0, exposure_time=0.1, size_x=3232, req_pct=100.0),
    dict(N=100, K=16, start_deg=0.0, stop_deg=1440.0, exposure_time=0.1, size_x=3232, req_pct=100.0),
]

if __name__ == "__main__":
    for p in PARAM_SETS:
        for mode in sorted(ANGLE_FUNCS):
            print_table(mode=mode, **p)
