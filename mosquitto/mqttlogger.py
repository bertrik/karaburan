#!/usr/bin/env python3

from __future__ import annotations

import os
from datetime import datetime, timezone
from typing import Any

import paho.mqtt.client as mqtt
from paho.mqtt.client import Client, MQTTMessage
from paho.mqtt.properties import Properties
from paho.mqtt.reasoncodes import ReasonCode

MQTT_HOST: str = os.getenv("MQTT_HOST", "localhost")
MQTT_PORT: int = int(os.getenv("MQTT_PORT", "1883"))
MQTT_TOPIC: str = os.getenv("MQTT_TOPIC", "karaburan/#")
LOG_DIR: str = os.getenv("LOG_DIR", "./mqttlog")


def current_log_file() -> str:
    """Return the log filename for the current 10-minute interval."""
    now: datetime = datetime.now(timezone.utc)

    minute: int = (now.minute // 10) * 10

    timestamp: datetime = now.replace(
        minute=minute,
        second=0,
        microsecond=0,
    )

    return os.path.join(
        LOG_DIR,
        f"{timestamp:%Y%m%dT%H%M}_mqtt.log",
    )


def log_message(topic: str, payload: bytes) -> None:
    """Append an MQTT message to the current log file."""
    filename: str = current_log_file()
    timestamp: str = datetime.now().isoformat(timespec="milliseconds")

    try:
        payload_text: str = payload.decode("utf-8")
    except UnicodeDecodeError:
        payload_text = payload.hex()

    with open(filename, "a", encoding="utf-8") as logfile:
        logfile.write(f"{timestamp} | {topic} | {payload_text}\n")


def on_connect(
    client: Client,
    _userdata: Any,
    _flags: mqtt.ConnectFlags,
    reason_code: ReasonCode,
    _properties: Properties | None,
) -> None:
    """Handle a successful or failed connection."""
    if reason_code.is_failure:
        print(f"MQTT connection failed: {reason_code}")
        return

    print(f"Connected to MQTT broker at {MQTT_HOST}:{MQTT_PORT}")

    result, _ = client.subscribe(MQTT_TOPIC)

    if result == mqtt.MQTT_ERR_SUCCESS:
        print(f"Subscribed to {MQTT_TOPIC}")
    else:
        print(f"Failed to subscribe: {mqtt.error_string(result)}")


def on_message(
    _client: Client,
    _userdata: Any,
    message: MQTTMessage,
) -> None:
    """Handle an incoming MQTT message."""
    log_message(message.topic, message.payload)


def on_disconnect(
    _client: Client,
    _userdata: Any,
    _disconnect_flags: mqtt.DisconnectFlags,
    reason_code: ReasonCode,
    _properties: Properties | None,
) -> None:
    """Handle disconnection from the MQTT broker."""
    print(f"Disconnected from MQTT broker: {reason_code}")


def main() -> None:
    """Start the MQTT logger."""
    os.makedirs(LOG_DIR, exist_ok=True)

    client: Client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    )

    client.on_connect = on_connect
    client.on_message = on_message
    client.on_disconnect = on_disconnect

    print("Starting MQTT logger...")
    print(f"Broker: {MQTT_HOST}:{MQTT_PORT}")
    print(f"Topic:  {MQTT_TOPIC}")
    print(f"Logs:   {LOG_DIR}")

    client.connect(
        MQTT_HOST,
        MQTT_PORT,
        keepalive=60,
    )

    client.loop_forever()


if __name__ == "__main__":
    main()
