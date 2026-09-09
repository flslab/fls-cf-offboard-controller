"""Acknowledged low-level to high-level commander ownership transfer.

The local Crazyflie firmware stops its high-level planner on every accepted
low-level setpoint. A stopped planner emits a null (motors-off) setpoint, so
relaxing low-level priority *before* starting a high-level plan is unsafe.
High-level commands run on a different firmware task from the priority
notification: merely reversing Python calls does not prove the plan started.

Caller contract: stop/join all low-level producers first, send their final
safe setpoint, and have no other in-flight high-level command. This helper
starts one plan, waits for its firmware success reply, then relaxes priority.
It never sends a low-level setpoint or retries a command: either would stop a
plan that might already be running. Callers own timeout/failure landing.

CRTP replies have no transaction ID and echo only three command bytes. The
single-flight lock below excludes concurrent calls through this helper, not
arbitrary cflib callers or an old delayed identical reply. After a timeout,
this CF object is latched untrusted and cannot attempt another handoff. Only
a new CF connection object clears that latch; there is no automatic reset.
A stale reply cannot be distinguished reliably without a firmware protocol
change, so no command retry may pretend it has a new reply identity.
"""
from __future__ import annotations

import inspect
import math
import struct
import threading
import time
import weakref

from cflib.crazyflie.high_level_commander import HighLevelCommander
from cflib.crtp.crtpstack import CRTPPort

from Interaction.command_wrapper import CommandWrapper


class HandoffError(RuntimeError):
    """A high-level plan was not confirmed, or priority transfer failed."""


_locks_guard = threading.Lock()
_locks = weakref.WeakKeyDictionary()
_untrusted_connections = weakref.WeakSet()


def _unwrap(commander):
    while isinstance(commander, CommandWrapper):
        commander = commander._wrapped_instance
    return commander


def _command_arguments(command_name, args, kwargs):
    if command_name not in ("go_to", "land"):
        raise HandoffError("handoff supports only go_to or land, never stop")
    try:
        bound = inspect.signature(getattr(HighLevelCommander, command_name)).bind(
            None, *args, **kwargs)
        bound.apply_defaults()
        values = dict(bound.arguments)
        if values["group_mask"] != HighLevelCommander.ALL_GROUPS:
            raise ValueError("handoff must address ALL_GROUPS; a skipped group can reply success")
        fields = ("x", "y", "z", "yaw", "duration_s") if command_name == "go_to" else (
            "absolute_height_m", "duration_s")
        for name in fields:
            if isinstance(values[name], bool) or not math.isfinite(float(values[name])):
                raise ValueError(name + " must be finite")
        if values["duration_s"] <= 0:
            raise ValueError("duration_s must be positive")
        if command_name == "land" and values["yaw"] is not None:
            if isinstance(values["yaw"], bool) or not math.isfinite(float(values["yaw"])):
                raise ValueError("yaw must be finite or None")
        return values
    except (TypeError, ValueError, OverflowError) as exc:
        raise HandoffError("invalid high-level handoff command: " + str(exc)) from exc


def _reply_prefix(command_name, values, cf):
    if command_name == "go_to":
        try:
            version = cf.platform.get_protocol_version()
            command = (HighLevelCommander.COMMAND_GO_TO if version < 8
                       else HighLevelCommander.COMMAND_GO_TO_2)
            return bytes((command, values["group_mask"], bool(values["relative"])))
        except Exception as exc:
            raise HandoffError("cannot determine the high-level wire protocol") from exc
    # The firmware echoes the first three bytes, then writes its result in
    # byte 3. LAND_2's third byte is the first byte of the target-height float.
    try:
        return struct.pack("<BBf", HighLevelCommander.COMMAND_LAND_2,
                           values["group_mask"], values["absolute_height_m"])[:3]
    except (TypeError, ValueError, OverflowError, struct.error) as exc:
        raise HandoffError("landing parameters cannot be represented on the wire") from exc


def handoff_to_high_level(low_level, high_level, command_name, *args,
                         ack_timeout_s=0.15, dry_run=False, **kwargs):
    """Start ``go_to``/``land``, require firmware ret=0, then notify once.

    Default acknowledgement deadline is 150 ms from just before command send;
    no automatic retry or intervening low-level keepalive is permitted. The
    callback is removed on every path. Callback/command/logging/timeout errors
    raise HandoffError and do not deliberately relax ownership.

    Once a high-level send was attempted, any unconfirmed transfer latches
    this CF object untrusted. This conservatively includes wrapper errors
    where the helper cannot establish whether a physical packet was sent.
    The caller must use its independent low-level landing fallback; this
    helper does not reset trust or retry on the same CF object.

    ``dry_run=True`` is explicit and allowed only when *both* supplied objects
    are CommandWrapper instances with execution=False. Their command logging
    still runs, but no physical send or fabricated acknowledgement occurs.
    Tests of active execution should provide a fake CF callback interface.
    """
    values = _command_arguments(command_name, args, kwargs)
    if (isinstance(ack_timeout_s, bool) or not isinstance(ack_timeout_s, (int, float))
            or not math.isfinite(ack_timeout_s) or not 0 < ack_timeout_s <= 0.15):
        raise HandoffError("ack_timeout_s must be positive and at most 0.15 seconds")
    if dry_run:
        if not all(isinstance(item, CommandWrapper) and item.execution is False
                   for item in (low_level, high_level)):
            raise HandoffError("dry_run requires two explicitly execution-disabled CommandWrappers")
        try:
            getattr(high_level, command_name)(*args, **kwargs)
            low_level.send_notify_setpoint_stop()
        except Exception as exc:
            raise HandoffError("dry-run handoff logging failed") from exc
        return dict(status="dry_run", command=command_name, acknowledged=False,
                    priority_released=False, elapsed_s=0.)

    if any(isinstance(item, CommandWrapper) and not item.execution
           for item in (low_level, high_level)):
        raise HandoffError("execution-disabled commanders require explicit dry_run=True")
    low_target, high_target = _unwrap(low_level), _unwrap(high_level)
    cf = getattr(high_target, "_cf", None)
    if cf is None or getattr(low_target, "_cf", None) is not cf:
        raise HandoffError("both commanders must belong to the same real or explicit fake CF")
    if not all(callable(getattr(cf, name, None)) for name in (
            "add_port_callback", "remove_port_callback")):
        raise HandoffError("CF acknowledgement callback interface is required")
    if callable(getattr(cf, "is_called_by_incoming_handler_thread", None)):
        if cf.is_called_by_incoming_handler_thread():
            raise HandoffError("cannot wait for an ACK on the CF incoming-handler thread")
    prefix = _reply_prefix(command_name, values, cf)
    try:
        with _locks_guard:
            if cf in _untrusted_connections:
                raise HandoffError("CF handoff connection is untrusted after an earlier failed transfer; "
                                   "do not retry on this connection")
            lock = _locks.setdefault(cf, threading.Lock())
    except TypeError as exc:
        raise HandoffError("CF must support weak references for handoff ownership") from exc
    if not lock.acquire(blocking=False):
        raise HandoffError("another acknowledged handoff is already active for this CF")

    received = threading.Event()
    answer = {}
    armed_at = None

    def acknowledge(packet):
        if armed_at is None or received.is_set():
            return
        try:
            payload = bytes(packet.data)
            if (packet.port != CRTPPort.SETPOINT_HL or getattr(packet, "channel", 0) != 0
                    or len(payload) != 4 or payload[:3] != prefix):
                return
        except (AttributeError, TypeError, ValueError):
            return
        answer.update(result=payload[3], received_at=time.monotonic())
        received.set()

    registered = False
    attempted = False
    completed = False
    started = time.monotonic()
    try:
        try:
            registered = True
            cf.add_port_callback(CRTPPort.SETPOINT_HL, acknowledge)
            armed_at = time.monotonic()
            attempted = True
            getattr(high_level, command_name)(*args, **kwargs)
            remaining = max(0., armed_at+ack_timeout_s-time.monotonic())
            if not received.wait(remaining) or answer["received_at"] > armed_at+ack_timeout_s:
                raise HandoffError("high-level handoff ACK timed out; low-level priority was not relaxed")
            if answer["result"] != 0:
                raise HandoffError("high-level planner rejected command with result " + str(answer["result"]))
        finally:
            if registered:
                cf.remove_port_callback(CRTPPort.SETPOINT_HL, acknowledge)
                registered = False
        # No call which sends a low-level setpoint is allowed between the
        # acknowledged planner startup above and this priority-only notice.
        low_level.send_notify_setpoint_stop()
        completed = True
        return dict(status="acknowledged", command=command_name,
                    acknowledged=True, priority_released=True,
                    elapsed_s=time.monotonic()-started)
    except HandoffError:
        raise
    except Exception as exc:
        raise HandoffError("high-level handoff failed; ownership transfer is unconfirmed") from exc
    finally:
        if attempted and not completed:
            with _locks_guard:
                _untrusted_connections.add(cf)
        lock.release()
