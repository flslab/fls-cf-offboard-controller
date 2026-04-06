"""
Peer TCP latency test.

Run on the receiver drone first:
    python3 Interaction/tests/peer_latency_test.py --role receiver

Then on the sender drone (or locally with --droneless):
    python3 Interaction/tests/peer_latency_test.py --role sender --peer-ip <receiver_ip>

With droneless flag (no physical drone required):
    python3 orchestrator.py --interaction --droneless --skip-confirm
    (uses peer_latency_test mission file)
"""

import argparse
import socket
import time

NUM_PACKETS = 1_000_000
PACKET_SIZE = 20  # bytes
DEFAULT_PORT = 5570


def run_receiver(port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('', port))
    server.listen(1)
    print(f"[Receiver] Listening on port {port}...")

    conn, addr = server.accept()
    print(f"[Receiver] Connected from {addr}")

    received = 0
    buf = b''
    first_arrival = None
    last_arrival = None

    try:
        while received < NUM_PACKETS:
            chunk = conn.recv(65536)
            if not chunk:
                break
            buf += chunk
            # Timestamp each complete packet boundary
            while len(buf) >= PACKET_SIZE:
                buf = buf[PACKET_SIZE:]
                t = time.perf_counter()
                if first_arrival is None:
                    first_arrival = t
                last_arrival = t
                received += 1
    finally:
        conn.close()
        server.close()

    avg_iat = (last_arrival - first_arrival) / (received - 1) * 1e6 if received > 1 else 0.0
    print(f"\n[Receiver] Results:")
    print(f"  Packets received             : {received:,}")
    print(f"  Total reception time         : {last_arrival - first_arrival:.4f} s")
    print(f"  Avg packet inter-arrival time: {avg_iat:.4f} us")


def run_sender(peer_ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"[Sender] Connecting to {peer_ip}:{port}...")
    sock.connect((peer_ip, port))
    print(f"[Sender] Connected. Sending {NUM_PACKETS:,} packets of {PACKET_SIZE} bytes each...")

    payload = b'\x00' * PACKET_SIZE

    s = time.perf_counter()
    for _ in range(NUM_PACKETS):
        sock.sendall(payload)
    e = time.perf_counter() - s

    sock.close()

    print(f"\n[Sender] Results:")
    print(f"  Total elapsed time       : {e:.4f} s")
    print(f"  Average per-packet time  : {e / NUM_PACKETS * 1e6:.4f} us")
    print(f"  Throughput               : {NUM_PACKETS * PACKET_SIZE / e / 1e6:.2f} MB/s")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Peer TCP latency test")
    parser.add_argument("--role", choices=["sender", "receiver"], required=True,
                        help="Run as sender or receiver")
    parser.add_argument("--peer-ip", type=str, default="127.0.0.1",
                        help="IP of the receiver (sender only)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"TCP port (default: {DEFAULT_PORT})")
    parser.add_argument("--droneless", action="store_true",
                        help="No-op flag for compatibility with orchestrator launch")
    args = parser.parse_args()

    if args.role == "receiver":
        run_receiver(args.port)
    else:
        run_sender(args.peer_ip, args.port)
