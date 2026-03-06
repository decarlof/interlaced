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


# ---------------- Aerotech PSO programming (dense stream for FPGA) ----------------

def program_PSO4FPGA(
    pso_pv="2bmb:TomoScanFPGA:PSOCommand.BOUT",
    controller_model="Ensemble",
    axis="X",
    encoder_input=3,
    rotation_speed_pv="2bmb:m102.VELO",
    rotation_pos_pv="2bmb:m102.VAL",
    max_rotation_speed=50,
    motor_speed=50,              # speed to restore after arming move
    rotation_start_new=0.0,       # arm reference position (deg)
    window_counts=11840158,       # allow pulses for this many encoder counts
    pulse_width_us=10,            # PSOPulseWidth in microseconds
    window_margin_counts=5
):
    """
    Programs PSO to output a pulse every encoder count (PSODISTANCE FIXED 1),
    but only while inside a PSO window of [0, window_counts] counts relative to arm.

    Assumes: forward direction, and that "arm reference" is at rotation_start_new (0 deg).
    """

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

    # Tracking encoder source (dense pulse stream uses this)
    epics.caput(pso_pv, f"PSOTRACK {axis} INPUT {int(encoder_input)}", wait=True, timeout=10)

    # Dense pulse stream: pulse every encoder count
    epics.caput(pso_pv, f"PSODISTANCE {axis} FIXED 1", wait=True, timeout=10)

    # Window uses the same encoder
    epics.caput(pso_pv, f"PSOWINDOW {axis} 1 INPUT {int(encoder_input)}", wait=True, timeout=10)

    # Window range in encoder counts relative to ARM position.
    # Here we want pulses from 0..window_counts (i.e., 0..360deg worth of encoder counts).
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


# ---------------- main: build positions and program both PSO + FPGA ----------------

def main():
    encoder_pulses_per_rotation = 11840158
    projection_images = 3000

    # indices into the dense (1-count) pulse stream
    step = encoder_pulses_per_rotation // projection_images  # 3946
    positions = (np.arange(projection_images, dtype=np.int64) * step).tolist()

    print(f"step={step}")
    print(f"first 10 positions={positions[:10]}")
    print(f"last position={positions[-1]} (< {encoder_pulses_per_rotation})")

    # 1) program PSO to output all encoder pulses during 0..360deg window
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
        window_counts=encoder_pulses_per_rotation,
        pulse_width_us=10,
    )

    # 2) program FPGA downselect table (positions -> delays -> BRAM)
    write_PSO_array(positions)


if __name__ == "__main__":
    main()
