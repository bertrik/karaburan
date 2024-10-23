#!/usr/bin/env python3

import argparse
import datetime
import json
import time

import paho.mqtt.client as mqtt
import serial

# Configure the serial port (make sure to replace 'COM3' with your actual serial port)
BAUD_RATE = 9600  # Baud rate (can be adjusted based on your device)
SENSOR_TYPE = 'distance'


def ping(ser):
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
    parser.add_argument("-d", "--device", help="The serial device",
                        default="/dev/ttyUSB0")
    parser.add_argument("-b", "--broker", help="The MQTT broker host name",
                        default="localhost")
    parser.add_argument("-s", "--sensor", help="The sensor id",
                        default="pinger")
    parser.add_argument("-i", "--interval", help="The publish interval (seconds)", default=1)
    args = parser.parse_args()

    # set up topic structure
    base_topic = f"karaburan/sensors/{SENSOR_TYPE}/{args.sensor}"
    config_topic = f"{base_topic}/config"
    measurement_topic = f"{base_topic}/measurement"
    config = {'type': SENSOR_TYPE, 'id': args.sensor, 'unit': 'mm', 'location': 'side'}

    # connect to mqtt
    client = mqtt.Client()
    client.will_set(config_topic, payload=None, retain=True)
    client.connect(args.broker)

    # Open the serial port
    with serial.Serial(args.device, BAUD_RATE, timeout=1) as ser:
        client.publish(config_topic, json.dumps(config), retain=True)
        while True:
            timestamp = datetime.datetime.now(datetime.UTC).isoformat()
            distance = ping(ser)
            if distance:
                measurement = {'type': SENSOR_TYPE, 'id': args.sensor, 'time': timestamp, 'value': distance}
                client.publish(measurement_topic, json.dumps(measurement))

            # next
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
