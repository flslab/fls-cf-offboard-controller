#!/usr/bin/env python3
"""
switch_network.py — FLS Swarm Network Mode Switcher (controller / laptop side)

Reads network configuration from swarm_manifest.yaml, SSHes into each drone
to schedule a network switch, prints instructions for the operator to join the
same network, then waits for confirmation before returning an updated manifest
(with corrected IPs) for use by the orchestrator.

Can be used standalone or imported by orchestrator.py.

Standalone usage:
    python3 switch_network.py [--mode {wifi,adhoc,bluetooth}]
    python3 switch_network.py --mode wifi   # revert all drones to managed wifi
"""

import argparse
import io
import sys
import time
import threading
import yaml

from fabric import Connection
from invoke.exceptions import CommandTimedOut

MANIFEST_FILE  = "swarm_manifest.yaml"
SWITCH_SCRIPT  = "network_switch.sh"
SWITCH_DELAY_S = 6   # seconds between SSH close and interface switch on drone


# ─── Manifest helpers ─────────────────────────────────────────────────────────

def load_manifest(path: str = MANIFEST_FILE) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def drone_wifi_ip(drone: dict) -> str:
    """Original WiFi IP used for initial SSH."""
    return drone["ip"]


def drone_adhoc_ip(drone: dict) -> str | None:
    """IP the drone should use in adhoc/bluetooth mode."""
    return drone.get("adhoc_ip")


def controller_adhoc_ip(manifest: dict, mode: str) -> str:
    """IP the controller (this machine) will use in adhoc/bluetooth mode."""
    net = manifest.get("network", {})
    if mode == "adhoc":
        return net.get("adhoc", {}).get("controller_ip", manifest["controller"]["ip"])
    if mode == "bluetooth":
        return net.get("bluetooth", {}).get("controller_ip", manifest["controller"]["ip"])
    return manifest["controller"]["ip"]


# ─── Per-drone SSH helpers ────────────────────────────────────────────────────

def _upload_switch_script(conn: Connection, work_dir: str):
    conn.put(SWITCH_SCRIPT, remote=f"{work_dir}/{SWITCH_SCRIPT}")
    conn.run(f"chmod +x {work_dir}/{SWITCH_SCRIPT}", timeout=5, warn=True)


def _ssh(drone: dict, work_dir: str, deferred_cmd: str, label: str) -> bool:
    """Upload switch script and schedule a deferred command on the drone."""
    try:
        conn = Connection(host=drone_wifi_ip(drone), user=drone["user"], connect_timeout=10)
        _upload_switch_script(conn, work_dir)
        wrapped = (
            f"nohup bash -c '{deferred_cmd}' "
            f"> /tmp/netswitch_{drone['id']}.log 2>&1 < /dev/null &"
        )
        conn.run(wrapped, timeout=4, pty=False, warn=True)
        print(f"  [{drone['id']}] {label}")
        return True
    except CommandTimedOut:
        print(f"  [{drone['id']}] {label}  (SSH timed out — expected)")
        return True
    except Exception as exc:
        print(f"  [{drone['id']}] ERROR: {exc}")
        return False


def schedule_adhoc(drone: dict, net_cfg: dict, work_dir: str) -> bool:
    adhoc_cfg  = net_cfg.get("adhoc", {})
    adhoc_ip   = drone_adhoc_ip(drone)
    if not adhoc_ip:
        print(f"  [{drone['id']}] WARN: no adhoc_ip configured — skipping")
        return False

    ssid    = adhoc_cfg.get("ssid",      "fls-adhoc")
    channel = adhoc_cfg.get("channel",   6)
    iface   = adhoc_cfg.get("interface", "wlan0")

    cmd = (
        f"sleep {SWITCH_DELAY_S} && "
        f"bash {work_dir}/{SWITCH_SCRIPT} adhoc {adhoc_ip} {ssid} {channel} {iface}"
    )
    return _ssh(drone, work_dir, cmd,
                f"Scheduled adhoc switch → IP={adhoc_ip}  SSID={ssid}  ch={channel}")


def schedule_bluetooth(drone: dict, net_cfg: dict, work_dir: str) -> bool:
    bt_cfg       = net_cfg.get("bluetooth", {})
    adhoc_ip     = drone_adhoc_ip(drone)
    ctrl_mac     = bt_cfg.get("controller_mac", "")
    bt_iface     = bt_cfg.get("interface", "hci0")

    if not adhoc_ip:
        print(f"  [{drone['id']}] WARN: no adhoc_ip configured — skipping")
        return False
    if not ctrl_mac:
        print(f"  [{drone['id']}] WARN: no controller_mac in network.bluetooth — skipping")
        return False

    cmd = (
        f"sleep {SWITCH_DELAY_S} && "
        f"bash {work_dir}/{SWITCH_SCRIPT} bluetooth {ctrl_mac} {adhoc_ip} {bt_iface}"
    )
    return _ssh(drone, work_dir, cmd,
                f"Scheduled BT-PAN → IP={adhoc_ip}  ctrl={ctrl_mac}")


def revert_to_wifi(drone: dict, work_dir: str) -> bool:
    iface = "wlan0"
    try:
        conn = Connection(host=drone_wifi_ip(drone), user=drone["user"], connect_timeout=10)
        _upload_switch_script(conn, work_dir)
        conn.run(f"bash {work_dir}/{SWITCH_SCRIPT} wifi {iface}",
                 timeout=20, pty=False, warn=True)
        print(f"  [{drone['id']}] Reverted to managed WiFi.")
        return True
    except Exception as exc:
        print(f"  [{drone['id']}] ERROR reverting WiFi: {exc}")
        return False


# ─── Operator instructions ────────────────────────────────────────────────────

def _box(lines: list[str]):
    width = max(len(l) for l in lines) + 4
    print("╔" + "═" * width + "╗")
    for l in lines:
        print("║  " + l.ljust(width - 2) + "  ║")
    print("╚" + "═" * width + "╝")


def print_adhoc_instructions(net_cfg: dict):
    adhoc_cfg = net_cfg.get("adhoc", {})
    ssid      = adhoc_cfg.get("ssid",          "fls-adhoc")
    channel   = adhoc_cfg.get("channel",        6)
    ctrl_ip   = adhoc_cfg.get("controller_ip", "10.0.0.100")
    iface     = adhoc_cfg.get("interface",      "wlan0")

    _box([
        "ACTION REQUIRED — Connect YOUR machine to the Ad-hoc network",
        "",
        f"  Network (SSID) : {ssid}",
        f"  Channel        : {channel}",
        f"  Your IP        : {ctrl_ip}   (subnet 255.255.255.0)",
        "",
        "  macOS (System Settings → Wi-Fi → Other Network…):",
        f"    Network name : {ssid}",
        f"    Security     : None",
        f"    Then: System Settings → Network → Wi-Fi → Details",
        f"          TCP/IP → Configure IPv4: Manually → {ctrl_ip}",
        "",
        "  Linux (NetworkManager):",
        f"    nmcli con add type wifi con-name fls-adhoc ifname {iface} ssid {ssid} -- \\",
        f"      wifi.mode adhoc wifi.channel {channel} \\",
        f"      ipv4.method manual ipv4.addresses {ctrl_ip}/24 ipv6.method disabled",
        f"    nmcli con up fls-adhoc",
    ])
    print()


def print_bluetooth_instructions(net_cfg: dict):
    bt_cfg   = net_cfg.get("bluetooth", {})
    ctrl_ip  = bt_cfg.get("controller_ip", "10.0.0.100")
    ctrl_mac = bt_cfg.get("controller_mac", "<your BT MAC>")

    _box([
        "ACTION REQUIRED — Set up Bluetooth PAN on YOUR machine",
        "",
        f"  Your BT MAC (set in manifest): {ctrl_mac}",
        f"  Your IP on the PAN           : {ctrl_ip}",
        "",
        "  macOS:",
        "    1. System Settings → General → Sharing → Internet Sharing",
        "    2. Share from: Wi-Fi (or Ethernet)  |  To computers via: Bluetooth PAN",
        "    3. Turn on Internet Sharing",
        "    4. Note your Bluetooth MAC:",
        "         System Settings → General → About → scroll to Bluetooth",
        "",
        "  Linux (bluez-tools):",
        "    sudo bt-pan --server &          # act as NAP",
        f"   sudo ip addr add {ctrl_ip}/24 dev bnep0",
        "",
        "  Drones will attempt to pair automatically in a few seconds.",
        "  Accept any pairing requests that appear.",
    ])
    print()


# ─── Core orchestration ───────────────────────────────────────────────────────

def apply_network_mode(manifest: dict, mode: str | None = None) -> dict:
    """
    Apply the given network mode across all drones.

    - Uploads network_switch.sh and schedules the switch on each drone via SSH.
    - Prints operator instructions.
    - Waits for the operator to press ENTER once their machine is on the new network.
    - Returns an updated deep-copy of the manifest whose IPs reflect the new network,
      ready to be pushed to drones and used by the orchestrator.

    If mode is None, reads from manifest['network']['mode'] (default: 'wifi').
    """
    if mode is None:
        mode = manifest.get("network", {}).get("mode", "wifi")

    drones   = manifest.get("drones", [])
    net_cfg  = manifest.get("network", {})
    work_dir = manifest["common"]["work_dir"]

    print(f"\n[NetworkSwitch] Mode: {mode}  ({len(drones)} drone(s))\n")

    # ── wifi: just revert synchronously, no IP changes ──────────────────────
    if mode == "wifi":
        threads = [threading.Thread(target=revert_to_wifi, args=(d, work_dir)) for d in drones]
        for t in threads: t.start()
        for t in threads: t.join()
        print("[NetworkSwitch] All drones returning to managed WiFi.")
        return manifest

    # ── adhoc / bluetooth: schedule deferred switches ────────────────────────
    if mode == "adhoc":
        threads = [threading.Thread(target=schedule_adhoc, args=(d, net_cfg, work_dir))
                   for d in drones]
    elif mode == "bluetooth":
        threads = [threading.Thread(target=schedule_bluetooth, args=(d, net_cfg, work_dir))
                   for d in drones]
    else:
        print(f"[NetworkSwitch] Unknown mode '{mode}' — nothing changed.")
        return manifest

    for t in threads: t.start()
    for t in threads: t.join()

    print(f"\n[NetworkSwitch] All drones will switch in ~{SWITCH_DELAY_S}s.\n")

    # Print operator instructions
    if mode == "adhoc":
        print_adhoc_instructions(net_cfg)
    else:
        print_bluetooth_instructions(net_cfg)

    input(f">>> Press ENTER once YOUR machine is connected to the {mode} network... ")
    print()

    # Build updated manifest with adhoc IPs
    updated = yaml.safe_load(yaml.dump(manifest))   # deep copy via round-trip

    ctrl_ip = controller_adhoc_ip(manifest, mode)
    updated["controller"]["ip"] = ctrl_ip
    print(f"[NetworkSwitch] Controller IP → {ctrl_ip}")

    for drone in updated.get("drones", []):
        a_ip = drone.get("adhoc_ip")
        if a_ip:
            drone["ip"] = a_ip
            print(f"[NetworkSwitch] {drone['id']} IP → {a_ip}")

    print("[NetworkSwitch] Network switch complete.\n")
    return updated


# ─── Standalone entry point ───────────────────────────────────────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Switch FLS swarm network mode (adhoc / bluetooth / wifi)"
    )
    parser.add_argument(
        "--mode", choices=["wifi", "adhoc", "bluetooth"],
        help="Override the mode specified in swarm_manifest.yaml",
    )
    parser.add_argument(
        "--manifest", default=MANIFEST_FILE,
        help=f"Path to manifest (default: {MANIFEST_FILE})",
    )
    args = parser.parse_args()

    manifest = load_manifest(args.manifest)
    mode     = args.mode or manifest.get("network", {}).get("mode", "wifi")

    apply_network_mode(manifest, mode)
    print("[NetworkSwitch] Done.")
