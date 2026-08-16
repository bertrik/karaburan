#!/usr/bin/env python3

# Copyright 2024 Bertrik Sikken <bertrik@sikken.nl>

import argparse
import json
import socket

from databus import DataBusClient

# Define the host and port for gpsd
HOST = 'localhost'
PORT = 2947
SENSOR_TYPE = 'location'


class GpsdClient:
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.file = self.sock.makefile()

    def connect(self, host: str, port: int) -> None:
        self.sock.connect((host, port))

        # Send the command to gpsd to watch for data
        self.sock.sendall(b'?WATCH={"enable":true,"json":true}\n')

    def poll(self) -> str:
        line = self.file.readline()
        if line:
            line = line.strip()
        return line

    def close(self) -> None:
        self.sock.close()


def main():
    """ The main entry point """
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    DataBusClient.add_arguments(parser)
    parser.add_argument("-i", "--id", help="Sensor id", default="rtkgps")
    args = parser.parse_args()

    # connect to gpsd
    gpsd_client = GpsdClient()
    gpsd_client.connect(HOST, PORT)
    try:
        client = DataBusClient(args.broker, "gnss", args.id)
        client.start()
        previousReport = None
        while True:
            # Get a JSON string
            line = gpsd_client.poll()
            if line is None:
                continue

            # Decode and parse the JSON data
            report = json.loads(line)

            # Check for gpsd connection to device, publish config to indicate our presence
            if report['class'] == 'DEVICES':
                print(f"{line}")

            # Time position reports
            if report['class'] == 'TPV':
                if previousReport and previousReport != report:
                    client.publish(previousReport)
                    print(f"{report}")
                previousReport = report
    finally:
        gpsd_client.close()


if __name__ == "__main__":
    main()
