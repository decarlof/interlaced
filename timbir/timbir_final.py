#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

from interlaced_scan import InterlacedScan


def main():

    N = 32
    K = 4

    scan = InterlacedScan(
        InterlacedRotationStart=0.0,
        InterlacedNumberOfRotation=K,
        InterlacedNumAnglesPerRotation=N,
    )

    # Generate multitimbir pattern
    scan.generate_interlaced_multitimbir()

    theta_interlaced = scan.theta_interlaced
    theta_monotonic = scan.theta_monotonic

    print("Total projections:", len(theta_interlaced))

    # ----------------------------------
    # Plot acquisition order vs angle
    # ----------------------------------
    plt.figure()
    plt.plot(theta_interlaced,
             np.arange(len(theta_interlaced)),
             marker='o',
             linestyle='None')
    plt.title("Multitimbir - Acquisition Order")
    plt.xlabel("Angle (deg)")
    plt.ylabel("Acquisition Index")
    plt.grid(True)
    plt.show()

    # ----------------------------------
    # Plot monotonic (sorted) angles
    # ----------------------------------
    plt.figure()
    plt.plot(theta_monotonic,
             np.arange(len(theta_monotonic)),
             marker='o',
             linestyle='None')
    plt.title("Multitimbir - Monotonic Angles")
    plt.xlabel("Angle (deg)")
    plt.ylabel("Sorted Index")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
