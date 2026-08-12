#!/usr/bin/env python3
import datetime
import json
from argparse import ArgumentParser
from typing import Any

import paho.mqtt.client as mqtt


class DataBusClient:
    def _on_connect(self, client: mqtt.Client, userdata: Any, flags: mqtt.ConnectFlags, reason_code: mqtt.ReasonCode,
                    properties: mqtt.Properties | None) -> None:
        config = {'type': self.sensor_type, 'id': self.sensor_id}
        self.client.publish(self.config_topic, json.dumps(config), retain=True)

    def __init__(
            self,
            broker: str,
            sensor_type: str,
            sensor_id: str,
    ) -> None:
        self.broker = broker
        self.sensor_type = sensor_type
        self.sensor_id = sensor_id

        self.client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
        )

        self.base_topic = f"karaburan/sensors/{sensor_type}/{sensor_id}"
        self.config_topic = f"{self.base_topic}/config"
        self.client.will_set(self.config_topic, payload=None, retain=True)

        # Callbacks
        self.client.on_connect = self._on_connect

    def start(self) -> None:
        self.client.connect(self.broker)
        self.client.loop_start()

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()

    def publish(self, data) -> None:
        measurement_topic = f"{self.base_topic}/measurement"
        now = round(datetime.datetime.now(datetime.UTC).timestamp(), 3)
        message = {'time': now, 'type': self.sensor_type, 'id': self.sensor_id, 'data': data}
        self.client.publish(measurement_topic, json.dumps(message))

    @staticmethod
    def add_arguments(parser: ArgumentParser):
        parser.add_argument("-b", "--broker", help="The MQTT broker host name",
                            default="localhost")
