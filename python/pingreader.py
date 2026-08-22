#!/usr/bin/env python3

# Copyright 2024 Bertrik Sikken <bertrik@sikken.nl>

import argparse
import time
from typing import Optional

import serial

from databus import DataBusClient

# Configure the serial port (make sure to replace 'COM3' with your actual serial port)
BAUD_RATE = 9600  # Baud rate (can be adjusted based on your device)


def ping(ser: object) -> Optional[int]:
    ser.write(b'\b0x01')
    # Read data from the serial port
    data = ser.read(ser.in_waiting)
    if data:
        sentence = data.decode('utf-8').strip()
        print("Data received:", sentence)
        if sentence.startswith("Gap"):
            number_str = sentence.replace("Gap=", "").replace("mm", "")
            return int(number_str)
    print("No data received")
    return None


def main():
    """ The main entry point """
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    DataBusClient.add_arguments(parser)
    parser.add_argument("-d", "--device", help="The serial device",
                        default="/dev/ttyUSB0")
    parser.add_argument("-s", "--sensor", help="The sensor id",
                        default="pinger")
    parser.add_argument("-i", "--interval", help="The publish interval (seconds)", default=1)
    args = parser.parse_args()

    # Open the serial port
    with serial.Serial(args.device, BAUD_RATE, timeout=1) as ser:
        client = DataBusClient(args.broker, "distance", args.id)
        client.start()
        while True:
            distance = ping(ser)
            if distance:
                client.publish(distance)

            # next
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
