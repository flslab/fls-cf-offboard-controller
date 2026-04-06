"""
Peer TCP latency test — uses the same ZMQ PUB/SUB + send_json mechanism as interactions.py
so results are directly comparable to live interaction latency.

Run on the receiver drone first:
    python3 Interaction/tests/peer_latency_test.py --role receiver

Then on the sender drone (or locally with --droneless):
    python3 Interaction/tests/peer_latency_test.py --role sender --peer-ip <receiver_ip>
"""

import argparse
import sys
import time
from pathlib import Path

import zmq

# Ensure project root is on sys.path when run directly as a script
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

NUM_PACKETS = 1_000_000
DEFAULT_PORT = 5570

# Match the JSON payload shape used in interactions.py:
#   pub_socket.send_json({"type": "push", "drone_id": ...,
#                         "accumulated_offset": [...], "push_start_time": ...})
PAYLOAD = {
    "type": "push",
    "drone_id": "lb6",
    "accumulated_offset": [0.0, 0.0, 0.0],
    "push_start_time": 0.0,
}


def run_receiver(port):
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.bind(f"tcp://*:{port}")
    print(f"[Receiver] Listening on port {port}...")

    # Block until the first message arrives (sender may need a moment to connect)
    sock.recv_json()
    first_arrival = time.perf_counter()
    last_arrival = first_arrival
    received = 1
    timed_out = False

    try:
        while received < NUM_PACKETS:
            if not sock.poll(timeout=10_000):  # 10s in ms
                timed_out = True
                break
            sock.recv_json()
            last_arrival = time.perf_counter()
            received += 1
    finally:
        sock.close()
        ctx.term()

    if timed_out:
        print(f"\n[Receiver] No packet for 10s — terminated early "
              f"({received:,}/{NUM_PACKETS:,} received, {NUM_PACKETS - received:,} lost).")

    total_time = last_arrival - first_arrival
    avg_iat = total_time / (received - 1) * 1e6 if received > 1 else 0.0

    print(f"\n[Receiver] Results:")
    print(f"  Packets received             : {received:,} / {NUM_PACKETS:,}"
          + (" (INCOMPLETE)" if timed_out else ""))
    print(f"  Total reception time         : {total_time:.4f} s")
    print(f"  Avg packet inter-arrival time: {avg_iat:.4f} us")


def run_sender(port):
    from Interaction.ConnectionHelper import TCPPeerPublisher
    pub = TCPPeerPublisher(port, unlimited_hwm=True)
    print(f"[Sender] Bound on :{port}. Waiting for receiver to connect...")
    time.sleep(1)  # give the SUB socket time to connect

    print(f"[Sender] Sending {NUM_PACKETS:,} packets...")

    s = time.perf_counter()
    for i in range(NUM_PACKETS):
        PAYLOAD["push_start_time"] = i   # vary field so ZMQ does not deduplicate
        pub.send_json(PAYLOAD)
    e = time.perf_counter() - s

    pub.close()

    print(f"\n[Sender] Results:")
    print(f"  Total elapsed time           : {e:.4f} s")
    print(f"  Avg per-packet send time     : {e / NUM_PACKETS * 1e6:.4f} us")
    print(f"  Throughput                   : {NUM_PACKETS / e:.0f} msgs/s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Peer ZMQ latency test (same transport as interactions.py)")
    parser.add_argument("--role", choices=["sender", "receiver"], required=True,
                        help="Run as sender or receiver")
    parser.add_argument("--peer-ip", type=str, default="127.0.0.1",
                        help="IP of the sender (receiver side does not need this)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"ZMQ port (default: {DEFAULT_PORT})")
    parser.add_argument("--droneless", action="store_true",
                        help="No-op flag for compatibility with orchestrator launch")
    args = parser.parse_args()

    if args.role == "receiver":
        run_receiver(args.port)
    else:
        run_sender(args.port)
