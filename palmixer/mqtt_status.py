# -*- coding: utf-8 -*-
"""MQTT telemetry for PALmixer motions.

Vendors the fail-open, lazy-connect MQTTPublisher / MQTTSubscriber pattern
from APS12_SAXSDaq (tools/mqtt_topics.py): paho-mqtt >= 2.0, callback API v2,
MQTT v5, JSON payloads. Telemetry must never break hardware control, so a
missing broker or missing paho-mqtt degrades to a silent no-op rather than
raising.

Topic scheme, under the shared "aps12/<beamline>/..." root:

    aps12/<beamline>/palmixer/state    QoS 0, retained  -- {"state": "IDLE"|"BUSY", ...}
    aps12/<beamline>/palmixer/motion   QoS 1             -- one message per phase of a command

Motion payload:
    {
        "action": "<command name>",
        "phase":  "started" | "success" | "failure",
        "ok":     bool,
        "detail": "<human-readable message>",
        "trace":  "<12-hex correlation id, shared by the started/success/failure trio>",
        "ts":     <epoch seconds>,
    }
"""

import json
import os
import time
import uuid

from . import config

QOS_STATE = 0
QOS_MOTION = 1

PHASE_STARTED = "started"
PHASE_SUCCESS = "success"
PHASE_FAILURE = "failure"


def new_trace():
    """A short correlation id linking a started/success/failure trio."""
    return uuid.uuid4().hex[:12]


def _topic(beamline, *parts):
    cfg = config.get_section("mqtt")
    prefix = cfg.get("prefix", "aps12")
    return "/".join([prefix, str(beamline)] + [str(p) for p in parts])


def state_topic(beamline):
    return _topic(beamline, "palmixer", "state")


def motion_topic(beamline):
    return _topic(beamline, "palmixer", "motion")


def motion_payload(action, phase, ok, detail="", trace=None):
    return {
        "action": action,
        "phase": phase,
        "ok": bool(ok),
        "detail": detail,
        "trace": trace or new_trace(),
        "ts": time.time(),
    }


def state_payload(state, action=None):
    return {"state": state, "action": action, "ts": time.time()}


class MQTTPublisher:
    """Best-effort JSON publisher. Lazy-connects; if paho or the broker is
    unavailable, publish() is a no-op (returns False) instead of raising, so
    telemetry never breaks robot/pump/motor control."""

    def __init__(self, host=None, port=None, client_id_prefix="palmixer"):
        cfg = config.get_section("mqtt")
        self.host = host or cfg.get("host", "localhost")
        self.port = int(port or cfg.get("port", 1883))
        self._client_id_prefix = client_id_prefix
        self._client = None
        self._failed = False  # once connect fails, stop retrying every publish

    def _ensure(self):
        if self._client is not None:
            return True
        if self._failed:
            return False
        try:
            import paho.mqtt.client as mqtt
            from paho.mqtt.enums import CallbackAPIVersion

            cid = "%s-%d" % (self._client_id_prefix, os.getpid())
            c = mqtt.Client(CallbackAPIVersion.VERSION2, client_id=cid,
                             protocol=mqtt.MQTTv5)
            c.connect(self.host, self.port, keepalive=60)
            c.loop_start()
            self._client = c
            return True
        except Exception as e:
            print("WARNING: MQTT connect failed (%s); telemetry disabled" % e)
            self._failed = True
            return False

    def publish(self, topic, payload, qos=QOS_MOTION, retain=False):
        if not self._ensure():
            return False
        try:
            data = payload if isinstance(payload, str) else json.dumps(payload)
            self._client.publish(topic, data, qos=qos, retain=retain)
            return True
        except Exception as e:
            print("WARNING: MQTT publish failed: %s" % e)
            return False

    def close(self):
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass
            self._client = None


class MQTTSubscriber:
    """Best-effort JSON subscriber companion to :class:`MQTTPublisher`.

    Lazy-connects; a missing broker or missing paho-mqtt is non-fatal (the
    callback simply never fires). Supports multiple topic subscriptions via
    ``subscribe()``. ``on_message(topic, payload)`` receives the
    already-JSON-decoded payload (or raw bytes if decoding fails).

    Network I/O runs on paho's background thread (loop_start); the callback
    fires there, NOT on the Qt main thread. GUI callers must hop back to the
    main thread (e.g. via a Qt signal) before touching widgets.
    """

    def __init__(self, host=None, port=None, client_id_prefix="palmixer-sub"):
        cfg = config.get_section("mqtt")
        self.host = host or cfg.get("host", "localhost")
        self.port = int(port or cfg.get("port", 1883))
        self._client_id_prefix = client_id_prefix
        self._client = None
        self._failed = False
        self._handlers = []  # list of (topic, on_message, qos)

    def _ensure(self):
        if self._client is not None:
            return True
        if self._failed:
            return False
        try:
            import paho.mqtt.client as mqtt
            from paho.mqtt.enums import CallbackAPIVersion

            cid = "%s-%d" % (self._client_id_prefix, os.getpid())
            c = mqtt.Client(CallbackAPIVersion.VERSION2, client_id=cid,
                             protocol=mqtt.MQTTv5)
            c.on_message = self._on_message
            c.connect(self.host, self.port, keepalive=60)
            for (topic, _cb, qos) in self._handlers:
                try:
                    c.subscribe(topic, qos=qos)
                except Exception:
                    pass
            c.loop_start()
            self._client = c
            return True
        except Exception as e:
            print("WARNING: MQTT subscriber connect failed (%s); subscribe disabled" % e)
            self._failed = True
            return False

    def _on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            payload = msg.payload
        for (topic, on_message, _qos) in self._handlers:
            if _topic_matches(topic, msg.topic):
                try:
                    on_message(msg.topic, payload)
                except Exception as e:
                    print("WARNING: MQTT on_message handler error for %s: %s" % (msg.topic, e))

    def subscribe(self, topic, on_message, qos=QOS_MOTION):
        """Register a callback for ``topic``. Multiple subscribes are allowed."""
        self._handlers.append((topic, on_message, qos))
        if self._client is not None:
            try:
                self._client.subscribe(topic, qos=qos)
            except Exception as e:
                print("WARNING: MQTT subscribe(%s) failed: %s" % (topic, e))

    def start(self):
        """Force the lazy connect now (otherwise it happens on first subscribe)."""
        return self._ensure()

    def close(self):
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass
            self._client = None


def _topic_matches(pattern, topic):
    """Match a single MQTT subscription pattern (+/#) against a concrete topic."""
    if pattern == topic:
        return True
    pp = pattern.split("/")
    tp = topic.split("/")
    for i, p in enumerate(pp):
        if p == "#":
            return True
        if i >= len(tp):
            return False
        if p != "+" and p != tp[i]:
            return False
    return len(pp) == len(tp)
