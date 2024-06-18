#!/usr/bin/env python3

import argparse
import json
import time
import paho.mqtt.client as mqtt


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
    base_topic = f"karaburan/sensors/temperature/{args.sensor}"
    config_topic = f"{base_topic}/config"
    config = {'name': 'measurer', 'unit': 'degree Celcius', 'location': 'underwater', 'contact': 'bertrik'}

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
        measurement = {'time': timestamp, 'type': 'temperature', 'id': args.sensor, 'value': value}

        # publish
        topic = f"{base_topic}/measurement"
        payload = json.dumps(measurement)
        print(f"Sending {payload} to {topic}")
        client.publish(topic, payload)

        # wait
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
