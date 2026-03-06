#!/bin/env python
# /APSshare/anaconda/x86_64/bin/python

import time
import epics
import numpy as np


# ---------------- FPGA BRAM programming ----------------

def writeRAM_memPulseSeq(all_delays):
    addrPV = "2bmbMZ1:SG:memPulseSeq_ADDR"
    dataPV = "2bmbMZ1:SG:memPulseSeq_DIN"
    clkPV  = "2bmbMZ1:SG:memPulseSeq_CLK_Signal"
    wrtPV  = "2bmbMZ1:SG:memPulseSeq_WRT_Signal"
    nPV    = "2bmbMZ1:SG:memPulseSeq_N"

    epics.caput(wrtPV, "1")
    time.sleep(.001)

    for i in range(len(all_delays)):
        epics.caput(addrPV, i, wait=True, timeout=1e6)
        epics.caput(dataPV, int(all_delays[i]), wait=True, timeout=1e6)
        time.sleep(.001)
        epics.caput(clkPV, "1!", wait=True, timeout=1e6)
        time.sleep(.001)

    epics.caput(wrtPV, "0")
    epics.caput(nPV, len(all_delays), wait=True, timeout=1e6)


def positions_counter_index_to_delays(positions_counter_index):
    if not positions_counter_index:
        return []

    delays = [int(positions_counter_index[0])]
    for i in range(len(positions_counter_index) - 1):
        delay = int(positions_counter_index[i+1]) - int(positions_counter_index[i]) - 1
        if delay < 0:
            raise ValueError("Trigger positions must be strictly increasing.")
        delays.append(delay)
    return delays


def write_PSO_array(positions_counter_index):
    delays = positions_counter_index_to_delays(positions_counter_index)
    writeRAM_memPulseSeq(delays)


# ---------------- index generation (PSO-pulse units) ----------------

def build_uniform_pso_pulse_indices(total_pso_pulses, projection_images):
    """
    Build a 0-based, strictly increasing list of indices into the PSO pulse stream.

    total_pso_pulses: total number of PSO pulses available in the window (e.g. 358789)
    projection_images: number of desired trigger pulses (e.g. 3000)
    """
    if projection_images <= 0:
        raise ValueError("projection_images must be > 0")
    if total_pso_pulses <= projection_images:
        raise ValueError("total_pso_pulses must be > projection_images")

    step = int(round(total_pso_pulses / projection_images))  # closest integer (e.g. 120)
    idx = np.arange(projection_images, dtype=np.int64) * step

    # keep last index inside [0, total_pso_pulses-1]
    overflow = int(idx[-1] - (total_pso_pulses - 1))
    if overflow > 0:
        idx = idx - overflow
        if idx[0] < 0:
            idx = np.round(np.linspace(0, total_pso_pulses - 1, projection_images)).astype(np.int64)

    # enforce strictly increasing (protect against rounding duplicates)
    for i in range(1, len(idx)):
        if idx[i] <= idx[i-1]:
            idx[i] = idx[i-1] + 1

    if idx[-1] >= total_pso_pulses:
        raise ValueError("Could not build valid index list inside total_pso_pulses.")

    return idx.tolist(), step


# ---------------- Aerotech PSO programming (PSODISTANCE FIXED N) ----------------

def program_PSO(
    pso_pv="2bmb:TomoScanFPGA:PSOCommand.BOUT",
    controller_model="Ensemble",
    axis="X",
    encoder_input=3,
    rotation_speed_pv="2bmb:m102.VELO",
    rotation_pos_pv="2bmb:m102.VAL",
    max_rotation_speed=50,
    motor_speed=50,
    rotation_start_new=0.0,
    window_counts=11840158,
    pulse_width_us=10,
    window_margin_counts=5,
    pso_distance=33,
):
    if controller_model != "Ensemble":
        raise ValueError("This function currently implements Ensemble syntax only.")

    # Move to arming reference position
    epics.caput(rotation_speed_pv, max_rotation_speed, wait=True, timeout=600)
    epics.caput(rotation_pos_pv, rotation_start_new, wait=True, timeout=600)
    epics.caput(rotation_speed_pv, motor_speed, wait=True, timeout=600)

    # Reset PSO
    epics.caput(pso_pv, f"PSOCONTROL {axis} RESET", wait=True, timeout=10)

    # Route PSO output to controller I/O terminal
    epics.caput(pso_pv, f"PSOOUTPUT {axis} CONTROL 1", wait=True, timeout=10)

    # Pulse width (microseconds)
    epics.caput(pso_pv, f"PSOPULSE {axis} TIME {pulse_width_us},{pulse_width_us}", wait=True, timeout=10)

    # Window-gated pulsing
    epics.caput(pso_pv, f"PSOOUTPUT {axis} PULSE WINDOW MASK", wait=True, timeout=10)

    # Tracking encoder source
    epics.caput(pso_pv, f"PSOTRACK {axis} INPUT {int(encoder_input)}", wait=True, timeout=10)

    # Set PSO distance (counts between PSO pulses)
    epics.caput(pso_pv, f"PSODISTANCE {axis} FIXED {int(pso_distance)}", wait=True, timeout=10)

    # Window uses the same encoder
    epics.caput(pso_pv, f"PSOWINDOW {axis} 1 INPUT {int(encoder_input)}", wait=True, timeout=10)

    # Window range in encoder counts relative to ARM position
    window_start = 0
    window_end = int(window_counts)
    epics.caput(
        pso_pv,
        f"PSOWINDOW {axis} 1 RANGE {window_start - window_margin_counts},{window_end + window_margin_counts}",
        wait=True,
        timeout=10
    )

    # Arm PSO
    epics.caput(pso_pv, f"PSOCONTROL {axis} ARM", wait=True, timeout=10)


# ---------------- main ----------------

def main():
    encoder_counts_per_rev = 11840158
    projection_images = 3000
    pso_distance = 33  # PSODISTANCE X FIXED 33

    # Program PSO
    program_PSO(
        pso_pv="2bmb:TomoScanFPGA:PSOCommand.BOUT",
        controller_model="Ensemble",
        axis="X",
        encoder_input=3,
        rotation_speed_pv="2bmb:m102.VELO",
        rotation_pos_pv="2bmb:m102.VAL",
        max_rotation_speed=50,
        motor_speed=50,
        rotation_start_new=0.0,
        window_counts=encoder_counts_per_rev,
        pulse_width_us=10,
        pso_distance=pso_distance,
    )

    # Compute total PSO pulses in the window (use the same logic as your counter)
    total_pso_pulses = encoder_counts_per_rev // pso_distance  # e.g. 358793 (you observed ~358789)

    # Build uniform indices into the PSO pulse stream
    positions, step = build_uniform_pso_pulse_indices(
        total_pso_pulses=total_pso_pulses,
        projection_images=projection_images
    )

    print(f"pso_distance={pso_distance} counts/PSO-pulse")
    print(f"total_pso_pulses={total_pso_pulses}")
    print(f"index step (rounded)={step} PSO-pulses")
    print(f"first 10 positions={positions[:10]}")
    print(f"last 10 positions={positions[-10:]}")
    print(f"last position={positions[-1]} (< {total_pso_pulses})")

    # Program FPGA downselect table
    write_PSO_array(positions)


if __name__ == "__main__":
    main()
