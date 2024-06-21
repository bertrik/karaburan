#!/usr/bin/env python3

import argparse
import socket
import json
import paho.mqtt.client as mqtt

# Define the host and port for gpsd
HOST = 'localhost'
PORT = 2947

class GpsdClient(object):
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.sock.connect((self.host, self.port))

        # Send the command to gpsd to watch for data
        self.sock.sendall(b'?WATCH={"enable":true,"json":true}\n')

    def poll(self):
        data = self.sock.recv(4096)
        if not data:
            return data
        data = data.decode('utf-8').strip()
        return data

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
    config = {'name': 'location', 'unit': 'lat/lon/alt', 'location': 'topside', 'contact': 'bertrik'}

    # connect to mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.will_set(config_topic, payload=None, retain=True)
    client.connect(args.broker)
    client.publish(config_topic, json.dumps(config), retain=True)

    # connect to gpsd
    gpsd_client = GpsdClient(HOST, PORT)
    gpsd_client.connect()

    try:
        while True:
            data = gpsd_client.poll()

            # Decode and parse the JSON data
            reports = data.split('\n')
            for report in reports:
                if report:
                    report = json.loads(report)

                    # publish
                    topic = f"{base_topic}/measurement"
                    client.publish(topic, json.dumps(report))

                    # Check for TPV reports
                    if report['class'] == 'TPV':
                        latitude = report.get('lat', None)
                        longitude = report.get('lon', None)
                        altitude = report.get('alt', None)
                        speed = report.get('speed', None)
                        time = report.get('time', None)

                        print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}, Speed: {speed}, Time: {time}")

    finally:
        gpsd_client.close()

if __name__ == "__main__":
    main()

