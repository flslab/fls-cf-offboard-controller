"""
Peer TCP latency test — uses the same ZMQ PUB/SUB + send_json mechanism as interactions.py
so results are directly comparable to live interaction latency.

Run on the receiver drone first:
    python3 Interaction/tests/peer_latency_test.py --role receiver

Then on the sender drone (or locally with --droneless):
    python3 Interaction/tests/peer_latency_test.py --role sender --peer-ip <receiver_ip>
"""

import argparse
import os
import platform
import sys
import threading
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
# PAYLOAD = b'\x00' * 20  # 20-byte fixed payload


# ---------------------------------------------------------------------------
# System monitor — samples CPU, memory, network, WiFi, TCP every second
# ---------------------------------------------------------------------------

class SystemMonitor:
    def __init__(self):
        self.samples = []
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True, name="sys-monitor")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3)

    # -- /proc readers -------------------------------------------------------

    @staticmethod
    def _read_cpu():
        try:
            with open('/proc/stat') as f:
                line = f.readline()
            vals = list(map(int, line.split()[1:8]))
            return vals[3], sum(vals)   # (idle, total)
        except Exception:
            return None, None

    @staticmethod
    def _read_mem():
        try:
            info = {}
            with open('/proc/meminfo') as f:
                for line in f:
                    k, v = line.split(':')
                    info[k.strip()] = int(v.split()[0])
            return info.get('MemTotal'), info.get('MemAvailable')
        except Exception:
            return None, None

    @staticmethod
    def _read_net():
        """Return stats for the first wlan* or eth* interface found."""
        try:
            with open('/proc/net/dev') as f:
                lines = f.readlines()
            for line in lines:
                if 'wlan' in line or 'eth' in line:
                    parts = line.split()
                    return {
                        'iface':    parts[0].rstrip(':'),
                        'rx_bytes': int(parts[1]),
                        'rx_pkts':  int(parts[2]),
                        'rx_errs':  int(parts[3]),
                        'rx_drop':  int(parts[4]),
                        'tx_bytes': int(parts[9]),
                        'tx_pkts':  int(parts[10]),
                        'tx_errs':  int(parts[11]),
                        'tx_drop':  int(parts[12]),
                    }
        except Exception:
            pass
        return None

    @staticmethod
    def _read_tcp():
        """Return RetransSegs and InErrs from /proc/net/snmp."""
        try:
            with open('/proc/net/snmp') as f:
                content = f.read()
            lines = [l for l in content.split('\n') if l.startswith('Tcp:')]
            keys = lines[0].split()[1:]
            vals = lines[1].split()[1:]
            d = dict(zip(keys, map(int, vals)))
            return d.get('RetransSegs', 0), d.get('InErrs', 0)
        except Exception:
            return None, None

    @staticmethod
    def _read_wifi():
        """Return link quality and signal level from /proc/net/wireless."""
        try:
            with open('/proc/net/wireless') as f:
                lines = f.readlines()
            for line in lines[2:]:
                if line.strip():
                    parts = line.split()
                    return {
                        'iface': parts[0].rstrip(':'),
                        'link':  float(parts[2].rstrip('.')),
                        'level': float(parts[3].rstrip('.')),
                        'noise': float(parts[4].rstrip('.')),
                    }
        except Exception:
            pass
        return None

    @staticmethod
    def _thread_count():
        try:
            return len(os.listdir(f'/proc/{os.getpid()}/task'))
        except Exception:
            return None

    # -- sampling loop -------------------------------------------------------

    def _run(self):
        prev_idle, prev_total = self._read_cpu()
        prev_net = self._read_net()
        prev_retrans, _ = self._read_tcp()
        t0 = time.perf_counter()

        while not self._stop.wait(1.0):
            t = time.perf_counter() - t0

            # CPU %
            idle, total = self._read_cpu()
            if None not in (prev_idle, idle) and total > prev_total:
                cpu_pct = 100.0 * (1.0 - (idle - prev_idle) / (total - prev_total))
            else:
                cpu_pct = None
            prev_idle, prev_total = idle, total

            # Memory %
            mem_total, mem_avail = self._read_mem()
            mem_pct = 100.0 * (1.0 - mem_avail / mem_total) if mem_total else None

            # Network delta
            net = self._read_net()
            net_d = None
            if net and prev_net and net['iface'] == prev_net['iface']:
                net_d = {
                    'iface':       net['iface'],
                    'rx_bytes_ps': net['rx_bytes'] - prev_net['rx_bytes'],
                    'rx_pkts_ps':  net['rx_pkts']  - prev_net['rx_pkts'],
                    'rx_errs':     net['rx_errs'],
                    'rx_drop':     net['rx_drop'],
                    'tx_bytes_ps': net['tx_bytes'] - prev_net['tx_bytes'],
                }
            prev_net = net

            # TCP retransmits delta
            retrans, in_errs = self._read_tcp()
            retrans_d = (retrans - prev_retrans) if None not in (retrans, prev_retrans) else None
            prev_retrans = retrans

            # WiFi
            wifi = self._read_wifi()

            self.samples.append({
                't':           t,
                'cpu_pct':     cpu_pct,
                'mem_pct':     mem_pct,
                'net':         net_d,
                'tcp_retrans': retrans_d,
                'tcp_in_errs': in_errs,
                'wifi':        wifi,
                'threads':     self._thread_count(),
            })

    # -- report --------------------------------------------------------------

    def report(self):
        if not self.samples:
            print("[Monitor] No samples collected.")
            return

        print("\n[Monitor] Per-second system samples:")
        hdr = (f"  {'t(s)':>6}  {'CPU%':>6}  {'Mem%':>6}  "
               f"{'RX MB/s':>8}  {'RX pkt/s':>10}  {'RX drop':>8}  "
               f"{'TCP retrans':>12}  {'WiFi link':>10}  {'Threads':>8}")
        print(hdr)
        print("  " + "-" * (len(hdr) - 2))

        for s in self.samples:
            cpu    = f"{s['cpu_pct']:.1f}"    if s['cpu_pct']  is not None else "N/A"
            mem    = f"{s['mem_pct']:.1f}"    if s['mem_pct']  is not None else "N/A"
            net    = s['net']
            rx_mb  = f"{net['rx_bytes_ps']/1e6:.2f}" if net else "N/A"
            rx_pkt = f"{net['rx_pkts_ps']}"          if net else "N/A"
            rx_drp = f"{net['rx_drop']}"              if net else "N/A"
            retrans = f"{s['tcp_retrans']}" if s['tcp_retrans'] is not None else "N/A"
            wifi   = s['wifi']
            wlink  = f"{wifi['link']:.0f} / {wifi['level']:.0f} dBm" if wifi else "N/A"
            thrs   = f"{s['threads']}" if s['threads'] else "N/A"
            print(f"  {s['t']:>6.1f}  {cpu:>6}  {mem:>6}  "
                  f"{rx_mb:>8}  {rx_pkt:>10}  {rx_drp:>8}  "
                  f"{retrans:>12}  {wlink:>10}  {thrs:>8}")


# ---------------------------------------------------------------------------
# Receiver
# ---------------------------------------------------------------------------

def run_receiver(port, sender_ip):
    # --- environment info ---------------------------------------------------
    print("\n[Receiver] === Environment ===")
    print(f"  Python       : {sys.version.split()[0]}")
    print(f"  ZMQ lib      : {zmq.zmq_version()}")
    print(f"  pyzmq        : {zmq.__version__}")
    print(f"  Platform     : {platform.platform()}")
    print(f"  CPU cores    : {os.cpu_count()}")
    mem_total, mem_avail = SystemMonitor._read_mem()
    if mem_total:
        print(f"  Memory       : {mem_total//1024} MB total, {mem_avail//1024} MB available")

    # --- socket setup -------------------------------------------------------
    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 0)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(f"tcp://{sender_ip}:{port}")

    actual_rcvhwm = sock.getsockopt(zmq.RCVHWM)
    print(f"\n[Receiver] === Socket Config ===")
    print(f"  Connecting to: tcp://{sender_ip}:{port}")
    print(f"  RCVHWM       : {actual_rcvhwm} (0 = unlimited)")
    print(f"  Expecting    : {NUM_PACKETS:,} packets")
    print(f"  Payload type : {'JSON' if isinstance(PAYLOAD, dict) else 'bytes'}, "
          f"size: {len(PAYLOAD) if isinstance(PAYLOAD, bytes) else 'varies'} bytes")

    # --- start monitor ------------------------------------------------------
    monitor = SystemMonitor()
    monitor.start()

    # --- wait for first packet ----------------------------------------------
    print(f"\n[Receiver] Waiting for first packet (10s timeout)...")
    if not sock.poll(timeout=10_000):
        print("[Receiver] No packet for 10s — sender never connected.")
        monitor.stop()
        monitor.report()
        sock.close()
        ctx.term()
        return

    sock.recv()
    first_arrival = time.perf_counter()
    last_arrival  = first_arrival
    received      = 1
    timed_out     = False

    # Per-second bucket tracking
    bucket_start   = first_arrival
    bucket_count   = 0
    per_sec_counts = []      # (elapsed_s, pkts_in_that_second)

    # Inter-arrival time tracking
    prev_t      = first_arrival
    max_gap_us  = 0.0
    max_gap_at  = 0          # received count when max gap occurred
    gaps_1ms    = 0          # gaps > 1 ms
    gaps_10ms   = 0          # gaps > 10 ms
    gaps_100ms  = 0          # gaps > 100 ms

    print(f"[Receiver] First packet arrived — receiving...")

    # --- receive loop -------------------------------------------------------
    try:
        while received < NUM_PACKETS:
            if not sock.poll(timeout=10_000):
                timed_out = True
                break

            sock.recv()
            t = time.perf_counter()
            last_arrival = t
            received += 1

            # IAT tracking
            gap_us = (t - prev_t) * 1e6
            if gap_us > max_gap_us:
                max_gap_us = gap_us
                max_gap_at = received
            if gap_us > 100_000:
                gaps_100ms += 1
            elif gap_us > 10_000:
                gaps_10ms += 1
            elif gap_us > 1_000:
                gaps_1ms += 1
            prev_t = t

            # Per-second bucket
            bucket_count += 1
            if t - bucket_start >= 1.0:
                per_sec_counts.append((t - first_arrival, bucket_count))
                bucket_count = 0
                bucket_start = t

    finally:
        # flush last bucket
        if bucket_count > 0:
            per_sec_counts.append((last_arrival - first_arrival, bucket_count))
        sock.close()
        ctx.term()

    monitor.stop()

    # --- results ------------------------------------------------------------
    total_time = last_arrival - first_arrival
    avg_iat    = total_time / (received - 1) * 1e6 if received > 1 else 0.0

    print(f"\n[Receiver] === Reception Results ===")
    print(f"  Packets received : {received:,} / {NUM_PACKETS:,}"
          + (f"  *** INCOMPLETE — {NUM_PACKETS - received:,} lost ({100*(NUM_PACKETS-received)/NUM_PACKETS:.1f}%) ***"
             if timed_out else ""))
    print(f"  Total time       : {total_time:.4f} s")
    print(f"  Avg IAT          : {avg_iat:.2f} us")
    print(f"  Max gap          : {max_gap_us:.0f} us  (after packet #{max_gap_at:,})")
    print(f"  Gaps > 1ms       : {gaps_1ms}")
    print(f"  Gaps > 10ms      : {gaps_10ms}")
    print(f"  Gaps > 100ms     : {gaps_100ms}")

    print(f"\n[Receiver] === Per-second Throughput ===")
    print(f"  {'Elapsed(s)':>12}  {'Pkts/s':>10}  {'MB/s (est)':>12}")
    payload_bytes = len(PAYLOAD) if isinstance(PAYLOAD, bytes) else 110
    for elapsed, count in per_sec_counts:
        print(f"  {elapsed:>12.1f}  {count:>10,}  {count * payload_bytes / 1e6:>12.2f}")

    monitor.report()


# ---------------------------------------------------------------------------
# Sender
# ---------------------------------------------------------------------------

def run_sender(port):
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 0)
    sock.bind(f"tcp://*:{port}")
    print(f"[Sender] Bound on :{port}. Waiting for receiver to connect...")
    time.sleep(1)  # give the SUB socket time to connect

    is_json = isinstance(PAYLOAD, dict)
    if is_json:
        import json
        payload_size = len(json.dumps(PAYLOAD).encode())
    else:
        payload_size = len(PAYLOAD)
    print(f"[Sender] Payload type: {'JSON' if is_json else 'bytes'}, size: {payload_size} bytes")
    print(f"[Sender] Sending {NUM_PACKETS:,} packets...")

    s = time.perf_counter()
    if is_json:
        for i in range(NUM_PACKETS):
            PAYLOAD["push_start_time"] = i
            sock.send_json(PAYLOAD)
    else:
        for _ in range(NUM_PACKETS):
            sock.send(PAYLOAD)
    e = time.perf_counter() - s

    sock.close()
    ctx.term()

    print(f"\n[Sender] Results:")
    print(f"  Total elapsed time           : {e:.4f} s")
    print(f"  Avg per-packet send time     : {e / NUM_PACKETS * 1e6:.4f} us")
    print(f"  Throughput                   : {NUM_PACKETS / e:.0f} msgs/s")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Peer ZMQ latency test (same transport as interactions.py)")
    parser.add_argument("--role", choices=["sender", "receiver"], required=True,
                        help="Run as sender or receiver")
    parser.add_argument("--peer-ip", type=str, default="127.0.0.1",
                        help="IP of the sender (required for receiver role)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help=f"ZMQ port (default: {DEFAULT_PORT})")
    parser.add_argument("--droneless", action="store_true",
                        help="No-op flag for compatibility with orchestrator launch")
    args = parser.parse_args()

    if args.role == "receiver":
        run_receiver(args.port, args.peer_ip)
    else:
        run_sender(args.port)