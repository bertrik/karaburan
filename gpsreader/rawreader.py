#!/usr/bin/env python3

import argparse
import socket
import time
import json
import paho.mqtt.client as mqtt

# Define the host and port for gpsd
HOST = 'localhost'
PORT = 2947

class GpsdClient():
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self, host, port):
        self.sock.connect((host, port))

        # Send the command to gpsd to watch for data
        self.sock.sendall(b'?WATCH={"enable":true,"json":true}\n')

    def poll(self):
        data = self.sock.recv(4096)
        if not data:
            return data
        data = data.decode('utf-8').strip()
        return data.split('\n')

    def close(self):
        self.sock.close()

def main():
    """ The main entry point """
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--sensor", help="Sensor id", default="1")
    parser.add_argument("-b", "--broker", help="The MQTT broker host name", default="localhost")
    args = parser.parse_args()

    # set up topic structure
    base_topic = f"karaburan/sensors/gps/{args.sensor}"
    config_topic = f"{base_topic}/config"
    config = {'name': 'location', 'unit': 'NA', 'location': 'topside', 'contact': 'bertrik'}

    # connect to mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.will_set(config_topic, payload=None, retain=True)
    client.connect(args.broker)

    # connect to gpsd
    gpsd_client = GpsdClient()
    gpsd_client.connect(HOST, PORT)
    try:
        while True:
            reports = gpsd_client.poll()

            # Decode and parse the JSON data
            for r in reports:
                report = json.loads(r)
                print(f"{report}")

                # Check for gpsd connection to device, publish config to indicate our presence
                if report['class'] == 'DEVICES':
                    config['devices'] = report['devices']
                    client.publish(config_topic, json.dumps(config), retain=True)

                # Time position reports
                if report['class'] == 'TPV':
                    topic = f"{base_topic}/measurement"
                    timestamp = time.time()
                    measurement = {'time': timestamp, 'type': 'gps', 'id': args.sensor, 'value': report}
                    client.publish(topic, json.dumps(measurement))
    finally:
        gpsd_client.close()

if __name__ == "__main__":
    main()

