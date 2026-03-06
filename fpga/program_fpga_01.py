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


# ---------------- Aerotech PSO programming (PSODISTANCE FIXED N) ----------------

def program_PSO4FPGA(
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
    pso_distance=33,  # <-- NEW: use 33 instead of 1
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

    # IMPORTANT: set PSO distance to 33 counts
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


# ---------------- main: build positions (in PSO-pulse units) and program PSO + FPGA ----------------

def main():
    encoder_counts_per_rev = 11840158
    projection_images = 3000

    # We are no longer using a dense stream (1 count/pulse).
    # PSO emits 1 pulse every N encoder counts. Indices for FPGA must be in units of *PSO pulses* now.
    pso_distance = 33  # counts per PSO pulse

    # Total number of PSO pulses over the window (~358,793)
    total_pso_pulses = encoder_counts_per_rev // pso_distance

    # We want 3000 triggers uniformly over those PSO pulses.
    step_pso = total_pso_pulses // projection_images  # ~119

    positions = (np.arange(projection_images, dtype=np.int64) * step_pso).tolist()

    print(f"pso_distance={pso_distance} counts/PSO-pulse")
    print(f"total_pso_pulses={total_pso_pulses}")
    print(f"step_pso={step_pso} PSO-pulses between triggers")
    print(f"first 10 positions={positions[:10]}")
    print(f"last position={positions[-1]} (< {total_pso_pulses})")

    # 1) program PSO with FIXED 33, windowed for one revolution
    program_PSO4FPGA(
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

    # 2) program FPGA downselect table (positions are PSO-pulse indices now)
    write_PSO_array(positions)


if __name__ == "__main__":
    main()
