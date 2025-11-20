#!/bin/env python

# /APSshare/anaconda/x86_64/bin/python

# examples of recorded macros, and recorded macros that have been edited
# to use arguments

import time
import epics
import numpy as np

def writeRAM_memPulseSeq(all):
    addrPV = "2bmbMZ1:SG:memPulseSeq_ADDR"        # BRAM addra (reg)
    dataPV = "2bmbMZ1:SG:memPulseSeq_DIN"          # BRAM dina  (reg)
    clkPV =  "2bmbMZ1:SG:memPulseSeq_CLK_Signal"   # BRAM clka
    wrtPV =  "2bmbMZ1:SG:memPulseSeq_WRT_Signal"   # BRAM wea
    enPV =   "2bmbMZ1:SG:memPulseSeq_EN_Signal"    # enable on memPulseSeq
    nPV =    "2bmbMZ1:SG:memPulseSeq_N"            # BRAM numPulses (reg)
    # note: BRAM ena signal (memPulseSeq_ENA) always 1.

    n = epics.caput(wrtPV, "1")
    # n = epics.caput(enPV, "0")
    time.sleep(.001)
    for i in range(0,len(all)):
        n = epics.caput(addrPV, i, wait=True, timeout=1000000.0)
        n = epics.caput(dataPV, all[i], wait=True, timeout=1000000.0)
        time.sleep(.001)
        n = epics.caput(clkPV,"1!", wait=True, timeout=1000000.0)
        time.sleep(.001)

    n = epics.caput(wrtPV, "0")
    # n = epics.caput(enPV, "1")
    n = epics.caput(nPV, len(all), wait=True, timeout=1000000.0)

def positions_to_delays(positions):
    """
    Convert an array of trigger positions into delays between pulses.

    positions: list or array of trigger indices (0-based, increasing)
               Example: [0, 3, 5]

    Returns a list of delays. Example: [0, 2, 1]
    """
    if not positions:
        return []

    delays = [positions[0]] # pulses before first trigger

    for i in range(len(positions) - 1):
        delay = positions[i+1] - positions[i] - 1
        if delay < 0:
            raise ValueError("Trigger positions must be strictly increasing.")
        delays.append(delay)

    return delays

def write_PSO_array(positions):
    delays = positions_to_delays(positions)
    writeRAM_memPulseSeq(delays)

