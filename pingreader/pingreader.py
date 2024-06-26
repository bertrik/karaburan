#!/usr/bin/env python3

import argparse
import json
import time

import paho.mqtt.client as mqtt
import serial

# Configure the serial port (make sure to replace 'COM3' with your actual serial port)
baud_rate = 9600  # Baud rate (can be adjusted based on your device)
sensor_type = 'distance'


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
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--device", help="The serial device",
                        default="/dev/ttyUSB0")
    parser.add_argument("-b", "--broker", help="The MQTT broker host name",
                        default="localhost")
    parser.add_argument("-s", "--sensor", help="The sensor id",
                        default="pinger")
    parser.add_argument("-i", "--interval", help="The publish interval (seconds)", default=1)
    args = parser.parse_args()

    # set up topic structure
    base_topic = f"karaburan/sensors/{sensor_type}/{args.sensor}"
    config_topic = f"{base_topic}/config"
    measurement_topic = f"{base_topic}/measurement"
    config = {'type': sensor_type, 'id': args.sensor, 'unit': 'mm', 'location': 'side', 'contact': 'bertrik'}

    # connect to mqtt
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.will_set(config_topic, payload=None, retain=True)
    client.connect(args.broker)

    # Open the serial port
    with serial.Serial(args.device, baud_rate, timeout=1) as ser:
        client.publish(config_topic, json.dumps(config), retain=True)
        while True:
            distance = ping(ser)
            if distance:
                measurement = {'type': sensor_type, 'id': args.sensor, 'value': distance}
                client.publish(measurement_topic, json.dumps(measurement))

            # next
            time.sleep(args.interval)


if __name__ == "__main__":
    main()
