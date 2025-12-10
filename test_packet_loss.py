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
    All drop events are logged to a CSV file.
    """

    # -----------------------------------------
    # Setup controller and network
    # -----------------------------------------
    leader = BILKLeader(debug=debug)
    leader.setup_network()

    print("Initializing encoders...")
    leader.read_encoders()

    RATE_HZ = 1000
    dt = 1.0 / RATE_HZ

    print("Running at {} Hz... Debug={}".format(RATE_HZ, debug))
    print("Packet loss enabled: {} (drop probability p={})".format(drop, p))
    print("------------------------------------------------------------")

    # -----------------------------------------
    # Create CSV log for drop events
    # -----------------------------------------
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_fname = "packet_loss_log_{}.csv".format(timestamp)
    log_file = open(log_fname, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t", "packet_index", "rand_val", "drop_prob"])

    print("[Logging] Packet-loss events -> {}".format(log_fname))

    # -----------------------------------------
    # Counters
    # -----------------------------------------
    sent = 0
    dropped = 0

    try:
        while True:

            # Read encoder data and build the UDP frame
            leader.read_encoders()
            frame = leader.build_frame()

            # -------------------------------------
            # PACKET LOSS INJECTION
            # -------------------------------------
            if drop:
                r = random.random()
                if r < p:
                    dropped += 1
                    sent += 1

                    t_now = time.time()
                    log.writerow(["{:.6f}".format(t_now), sent, r, p])

                    print("[DROP] packet={}, t={:.6f}, rand={:.3f} < p={}".format(
                        sent, t_now, r, p
                    ))

                    time.sleep(dt)
                    continue  # skip sending the UDP packet

            # -------------------------------------
            # NORMAL FRAME SENDING
            # -------------------------------------
            leader.send_frame(frame)
            sent += 1

            # Keep the update rate consistent
            time.sleep(dt)

            # Periodic summary
            if sent % 5000 == 0:
                drop_rate = float(dropped) / float(sent) if sent > 0 else 0.0
                print("[Packet Stats] sent={}, dropped={} ({:.2f}%)".format(
                    sent, dropped, drop_rate * 100.0
                ))

    except KeyboardInterrupt:
        print("\nStopping controller...")

        if leader.socket:
            leader.socket.close()

        log_file.close()

        drop_rate = float(dropped) / float(sent) if sent > 0 else 0.0
        print("\nFinal packet stats: sent={}, dropped={} ({:.2f}%)".format(
            sent, dropped, drop_rate * 100.0
        ))
        print("Drop log saved to: {}".format(log_fname))
        print("Controller stopped.\n")


def main():
    parser = argparse.ArgumentParser(description="Controller with random packet loss injection.")
    parser.add_argument("--debug", action="store_true",
                        help="Use mock encoder data.")
    parser.add_argument("--drop", action="store_true",
                        help="Enable packet loss injection.")
    parser.add_argument("--p", type=float, default=0.1,
                        help="Probability of dropping each packet (default 0.1)")

    args = parser.parse_args()

    run_with_packet_loss(
        debug=args.debug,
        drop=args.drop,
        p=args.p
    )


if __name__ == "__main__":
    main()
