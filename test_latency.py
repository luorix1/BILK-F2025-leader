#!/usr/bin/env python3

import time
import argparse
from controller import BILKLeader


def run_with_latency(debug=False, spike=False, spike_after=0.0, spike_duration=1.0):
    """
    Behaves exactly like controller.py but optionally injects a UDP silence window.
    """

    leader = BILKLeader(debug=debug)
    leader.setup_network()

    print("Initializing encoders...")
    leader.read_encoders()

    RATE_HZ = 1000
    dt = 1.0 / RATE_HZ

    print(f"Running at {RATE_HZ} Hz... Debug={debug}")
    print(f"Latency spike enabled: {spike} (after {spike_after}s, duration {spike_duration}s)")
    print("------------------------------------------------------------")

    start_time = time.time()
    spike_started = False
    spike_finished = False

    try:
        while True:
            t_now = time.time()
            elapsed = t_now - start_time

            # Read encoder data
            leader.read_encoders()
            frame = leader.build_frame()

            # --------------------------
            # LATENCY SPIKE MANAGEMENT
            # --------------------------
            if spike and not spike_started and elapsed >= spike_after:
                print("\n=== BEGINNING LATENCY SPIKE (no packets sent) ===\n")
                spike_started = True
                spike_end_time = t_now + spike_duration

            if spike_started and not spike_finished:
                if time.time() < spike_end_time:
                    # SKIP SENDING PACKETS
                    time.sleep(dt)
                    continue
                else:
                    print("\n=== LATENCY SPIKE OVER, RESUMING NORMAL TRANSMISSION ===\n")
                    spike_finished = True

            # --------------------------
            # NORMAL SEND BEHAVIOR
            # --------------------------
            leader.send_frame(frame)

            # keep consistent update rate
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nStopping controller...")
        if leader.socket:
            leader.socket.close()


def main():
    parser = argparse.ArgumentParser(description="Controller with optional latency injection.")
    parser.add_argument("--debug", action="store_true", help="Use mock encoder data.")
    parser.add_argument("--spike", action="store_true", help="Inject a 1s latency spike.")
    parser.add_argument("--spike-after", type=float, default=3.0,
                        help="Seconds after start to inject spike (default=3).")
    parser.add_argument("--spike-duration", type=float, default=1.0,
                        help="Latency spike duration in seconds (default=1).")

    args = parser.parse_args()

    run_with_latency(
        debug=args.debug,
        spike=args.spike,
        spike_after=args.spike_after,
        spike_duration=args.spike_duration
    )


if __name__ == "__main__":
    main()
