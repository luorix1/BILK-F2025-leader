#!/usr/bin/env python3

import time
import random
import argparse
import csv
from datetime import datetime
from controller import BILKLeader


def run_with_packet_loss(debug=False, drop=False, p=0.1):
    """
    Behaves like controller.py but optionally injects random packet loss.
    Also logs every drop event to a CSV file.
    """

    # ---------------------------
    # Set up leader + network
    # ---------------------------
    leader = BILKLeader(debug=debug)
    leader.setup_network()

    print("Initializing encoders...")
    leader.read_encoders()

    RATE_HZ = 1000
    dt = 1.0 / RATE_HZ

    print(f"Running at {RATE_HZ} Hz... Debug={debug}")
    print(f"Packet loss enabled: {drop} (drop probability p={p})")
    print("------------------------------------------------------------")

    # ---------------------------
    # Create drop log
    # ---------------------------
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_fname = f"packet_loss_log_{timestamp}.csv"
    log_file = open(log_fname, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t", "packet_index", "rand_val", "drop_prob"])

    print(f"[Logging] Packet-loss events → {log_fname}")

    # ---------------------------
    # Counters
    # ---------------------------
    sent = 0
    dropped = 0

    try:
        while True:

            # Read encoder data & build UDP frame
            leader.read_encoders()
            frame = leader.build_frame()

            # ------------------------------------
            # PACKET LOSS INJECTION
            # ------------------------------------
            if drop:
                r = random.random()
                if r < p:
                    dropped += 1
                    sent += 1

                    # Log the drop event
                    t_now = time.time()
                    log.writerow([f"{t_now:.6f}", sent, r, p])

                    # Debug print
                    print(f"[DROP] packet={sent}, t={t_now:.6f}, rand={r:.3f} < p={p}")

                    time.sleep(dt)
                    continue  # skip sending packet

            # ------------------------------------
            # NORMAL SEND BEHAVIOR
            # ------------------------------------
            leader.send_frame(frame)
            sent += 1

            # maintain update rate
            time.sleep(dt)

            # Summary every 5000 packets
            if sent % 5000 == 0:
                drop_rate = dropped / sent if sent > 0 else 0.0
                print(f"[Packet Stats] sent={sent}, dropped={dropped} ({drop_rate*100:.2f}%)")

    except KeyboardInterrupt:
        print("\nStopping controller...")

        if leader.socket:
            leader.socket.close()

        log_file.close()

        drop_rate = dropped / sent if sent > 0 else 0.0
        print(f"\nFinal packet stats: sent={sent}, dropped={dropped} ({drop_rate*100:.2f}%)")
        print(f"Drop log saved to: {log_fname}")
        print("Controller stopped.\n")


def main():
    parser = argparse.ArgumentParser(description="Controller with random packet loss injection.")
    parser.add_argument("--debug", action="store_true",
                        help="Use mock encoder data instead of hardware.")
    parser.add_argument("--drop", action="store_true",
                        help="Enable packet loss injection.")
    parser.add_argument("--p", type=float, default=0.1,
                        help="Probability of dropping each packet (default 0.1).")

    args = parser.parse_args()

    run_with_packet_loss(
        debug=args.debug,
        drop=args.drop,
        p=args.p
    )


if __name__ == "__main__":
    main()
