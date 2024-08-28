#!/usr/bin/env python3

import argparse
import socket
import time
import datetime
import json
import paho.mqtt.client as mqtt

# Define the host and port for gpsd
HOST = 'localhost'
PORT = 2947
SENSOR_TYPE = 'location'


class GpsdClient:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.file = self.sock.makefile()

    def connect(self, host, port):
        self.sock.connect((host, port))

        # Send the command to gpsd to watch for data
        self.sock.sendall(b'?WATCH={"enable":true,"json":true}\n')

    def poll(self):
        line = self.file.readline()
        if line:
            line = line.strip()
        return line

    def close(self):
        self.sock.close()


def main():
    """ The main entry point """
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("-s", "--sensor", help="Sensor id", default="1")
    parser.add_argument("-b", "--broker", help="The MQTT broker host name", default="localhost")
    args = parser.parse_args()

    # set up topic structure
    base_topic = f"karaburan/sensors/{SENSOR_TYPE}/{args.sensor}"
    config_topic = f"{base_topic}/config"
    measurement_topic = f"{base_topic}/measurement"
    config = {'type': SENSOR_TYPE, 'id': args.sensor, 'unit': 'NA', 'location': 'topside'}

    # connect to mqtt
    client = mqtt.Client()
    client.will_set(config_topic, payload=None, retain=True)
    client.connect(args.broker)

    # connect to gpsd
    gpsd_client = GpsdClient()
    gpsd_client.connect(HOST, PORT)
    try:
        while True:
            # Get a JSON string
            line = gpsd_client.poll()
            if line is None:
                continue

            # Decode and parse the JSON data
            timestamp = datetime.datetime.now(datetime.UTC).isoformat()
            report = json.loads(line)

            # Check for gpsd connection to device, publish config to indicate our presence
            if report['class'] == 'DEVICES':
                print(f"{line}")
                config['devices'] = report['devices']
                client.publish(config_topic, json.dumps(config), retain=True)

            # Time position reports
            if report['class'] == 'TPV':
                print(f"{line}")
                measurement = {'type': SENSOR_TYPE, 'id': args.sensor, 'time': timestamp, 'value': report}
                client.publish(measurement_topic, json.dumps(measurement))
    finally:
        gpsd_client.close()


if __name__ == "__main__":
    main()
