#!/usr/bin/env python3

import argparse
import json
import time
import paho.mqtt.client as mqtt

SENSOR_TYPE = 'temperature'


def read_temperature(sensor_id):
    sensor_file = f"/mnt/1wire/uncached/{sensor_id}/temperature"
    try:
        with open(sensor_file, 'r', encoding="UTF-8") as file:
            temperature = file.read().strip()
            return temperature
    except FileNotFoundError:
        print(f"Sensor {sensor_id} not found.")
        return None


def main():
    """ The main entry point """
    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--sensor", help="The temperature sensor id",
                        default="28.7AAB46D42000")
    parser.add_argument("-b", "--broker", help="The MQTT broker host name",
                        default="localhost")
    parser.add_argument("-i", "--interval", help="The publish interval (seconds)", default=15)
    args = parser.parse_args()

    # set up topic structure
    base_topic = f"karaburan/sensors/{SENSOR_TYPE}/{args.sensor}"
    config_topic = f"{base_topic}/config"
    measurement_topic = f"{base_topic}/measurement"
    config = {'type': SENSOR_TYPE, 'id': args.sensor, 'unit': 'degC', 'location': 'underwater'}

    # connect
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.will_set(config_topic, payload=None, retain=True)
    client.connect(args.broker)
    client.publish(config_topic, json.dumps(config), retain=True)

    while True:
        # get value
        timestamp = time.time()
        value = read_temperature(args.sensor)

        # build measurement structure
        measurement = {'type': SENSOR_TYPE, 'id': args.sensor, 'time': timestamp, 'value': value}

        # publish
        payload = json.dumps(measurement)
        print(f"Sending {payload} to {measurement_topic}")
        client.publish(measurement_topic, payload)

        # wait
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
