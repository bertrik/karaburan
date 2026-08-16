#!/usr/bin/env python3

# Copyright 2024 Bertrik Sikken <bertrik@sikken.nl>

import argparse
import time
from pathlib import Path

from databus import DataBusClient

OWFS_PATH = Path("/mnt/1wire/uncached")


def discover_sensors() -> list[str]:
    sensors = []
    for sensor in OWFS_PATH.iterdir():
        if not sensor.is_dir() or not sensor.name.startswith("28"):
            continue
        try:
            float((sensor / "temperature").read_text().strip())
            sensors.append(sensor.name)
        except (FileNotFoundError, ValueError, OSError):
            pass
    return sensors


def read_temperature(sensor_id: str) -> float | None:
    sensor_file = f"/mnt/1wire/uncached/{sensor_id}/temperature"
    try:
        with open(sensor_file, 'r', encoding="UTF-8") as file:
            temperature = file.read().strip()
            return float(temperature)
    except FileNotFoundError:
        print(f"Sensor {sensor_id} not found.")
        return None


def main():
    """ The main entry point """
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    DataBusClient.add_arguments(parser)
    parser.add_argument("-i", "--interval", help="The publish interval (seconds)", default=10)
    args = parser.parse_args()

    # discover temperature sensors
    sensor_ids = discover_sensors()
    print(f"Discovered sensors: {sensor_ids}")

    # connect
    clients = {}
    for id in sensor_ids:
        clients[id] = DataBusClient(args.broker, "temperature", id)
        clients[id].start()

    while True:
        for id, client in clients.items():
            # get value
            value = read_temperature(id)

            # publish
            print(f"Publising {value} for {id}")
            client.publish(value)

        # wait
        time.sleep(args.interval)


if __name__ == "__main__":
    main()
