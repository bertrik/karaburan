#!/usr/bin/env python3

""" receives position reports from MQTT and makes them available in a callback """

import argparse
import json
import time

import paho.mqtt.client as mqtt


class PositionReporter:
    def __init__(self, broker, topic, callback):
        self.broker = broker
        self.topic = topic
        self.callback = callback
        self.client = mqtt.Client()
        self.client.on_message = self.on_message
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.last_value = ""

    def start(self) -> None:
        print(f"Connecting to MQTT broker {self.broker}...")
        self.client.connect(self.broker)
        self.client.loop_start()

    def stop(self) -> None:
        self.client.disconnect()

    def on_connect(self, _client, _userdata, _flags, _rc) -> None:
        print(f"Connected, subscribing to topic {self.topic}...")
        self.client.subscribe(self.topic)

    def on_disconnect(self, _client, _userdata, rc) -> None:
        print(f"Disconnected with code {rc}, waiting for reconnect ...")

    def on_message(self, _client, _userdata, msg):
        payload = json.loads(msg.payload)
        value = payload["value"]
        if self.last_value != value:
            self.last_value = value
            self.callback(value)


def position_callback(measurement: dict) -> None:
    lat = measurement.get("lat")
    lon = measurement.get("lon")
    speed = measurement.get("speed")  # speed over ground (m/s)
    track = measurement.get("track")  # course over ground (degrees)
    eph = measurement.get("eph")  # horizontal position error (m)
    eps = measurement.get("eps")  # speed error (m/s)
    print(f"lat={lat:.8f},lon={lon:.8f},speed={speed:6.3f},track={track:5.1f},eph={eph:.3f},eps={eps:.3f}")


def main():
    """ The main entry point """
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--broker", help="The MQTT broker host name", default="localhost")
    parser.add_argument("--topic", help="The GPS topic", default="karaburan/sensors/location/1/measurement")
    args = parser.parse_args()

    # Start the position reporter
    reporter = PositionReporter(args.broker, args.topic, position_callback)
    reporter.start()
    try:
        while True:
            # do nothing, position reports arrive in the position_callback
            time.sleep(.1)
    except:
        reporter.stop()


if __name__ == "__main__":
    main()
