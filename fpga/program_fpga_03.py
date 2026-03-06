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
    if projection_images <= 0:
        raise ValueError("projection_images must be > 0")
    if total_pso_pulses <= projection_images:
        raise ValueError("total_pso_pulses must be > projection_images")

    step = int(round(total_pso_pulses / projection_images))
    idx = np.arange(projection_images, dtype=np.int64) * step

    overflow = int(idx[-1] - (total_pso_pulses - 1))
    if overflow > 0:
        idx = idx - overflow
        if idx[0] < 0:
            idx = np.round(np.linspace(0, total_pso_pulses - 1, projection_images)).astype(np.int64)

    for i in range(1, len(idx)):
        if idx[i] <= idx[i-1]:
            idx[i] = idx[i-1] + 1

    if idx[-1] >= total_pso_pulses:
        raise ValueError("Could not build valid index list inside total_pso_pulses.")

    return idx.tolist(), step


# ---------------- Aerotech PSO programming ----------------

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

    # Set PSO distance
    epics.caput(pso_pv, f"PSODISTANCE {axis} FIXED {int(pso_distance)}", wait=True, timeout=10)

    # Window uses same encoder
    epics.caput(pso_pv, f"PSOWINDOW {axis} 1 INPUT {int(encoder_input)}", wait=True, timeout=10)

    # Window range relative to ARM position
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


# ---------------- FPGA gating / counter reset ----------------

def fpga_reset_counters_and_enable(
    reset_pv="2bmbMZ1:SG:BUFFER-1_IN_Signal",
    enable_pv="2bmbMZ1:SG:BUFFER-2_IN_Signal",
    settle_s=1,
):
    """
    Per your requirement:
      1) reset counters by setting BUFFER-1_IN_Signal to 0
      2) enable trigger by transitioning BUFFER-2_IN_Signal 0 -> 1
    """
    # Reset counters
    epics.caput(reset_pv, "0", wait=True)
    time.sleep(settle_s)
    epics.caput(reset_pv, "1!", wait=True)

    # Force known low then rising edge to enable
    epics.caput(enable_pv, "0", wait=True)
    time.sleep(settle_s)
    epics.caput(enable_pv, "1", wait=True)
    time.sleep(settle_s)


# ---------------- main ----------------

def main():
    encoder_counts_per_rev = 11840158
    projection_images = 3000
    pso_distance = 33  # PSODISTANCE X FIXED 33

    # Program PSO (includes move to start position and ARM)
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

    # Build FPGA index list in PSO-pulse units
    total_pso_pulses = encoder_counts_per_rev // pso_distance  # ~358793
    positions, step = build_uniform_pso_pulse_indices(total_pso_pulses, projection_images)

    print(f"total_pso_pulses={total_pso_pulses}, projection_images={projection_images}, step~={step}")
    print(f"first 10 positions={positions[:10]}")
    print(f"last position={positions[-1]} (< {total_pso_pulses})")

    # Program FPGA BRAM
    write_PSO_array(positions)

    # After stage reached start position: reset counters and enable trigger
    fpga_reset_counters_and_enable(
        reset_pv="2bmbMZ1:SG:BUFFER-1_IN_Signal",
        enable_pv="2bmbMZ1:SG:BUFFER-2_IN_Signal"
    )


if __name__ == "__main__":
    main()
