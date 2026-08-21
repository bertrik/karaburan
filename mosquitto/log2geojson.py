#!/usr/bin/env python3

import argparse
import json
import re
from pathlib import Path


def parse_line(line):
    parts = line.strip().split("|", 2)

    if len(parts) != 3:
        raise ValueError("Expected: timestamp | topic | JSON")

    log_timestamp = parts[0].strip()
    topic = parts[1].strip()
    measurement = json.loads(parts[2].strip())

    return log_timestamp, topic, measurement


def sanitize_name(value):
    value = str(value)

    # Replace anything other than letters, numbers and _
    # with an underscore.
    value = re.sub(
        r"[^A-Za-z0-9_]",
        "_",
        value,
    )
    return value


def flatten_data(data, prefix):
    result = {}

    def flatten(value, name):

        if isinstance(value, dict):
            for key, child in value.items():
                flatten(
                    child,
                    f"{name}_{sanitize_name(key)}",
                )
        else:
            result[name] = value

    flatten(data, prefix)

    return result


def get_sensor_key(measurement):
    sensor_type = measurement.get(
        "type",
        "measurement",
    )

    sensor_id = measurement.get(
        "id",
        "unknown",
    )

    return (
        str(sensor_type),
        str(sensor_id),
    )


def add_latest_measurement(
        latest_measurements,
        measurement,
        topic,
        log_timestamp,
):
    """
    Store the latest measurement for each unique sensor.

    Sensor identity is based on:
        (type, id)
    Therefore two temperature sensors with different IDs
    are kept separately.
    """
    sensor_type, sensor_id = get_sensor_key(measurement)

    latest_measurements[(sensor_type, sensor_id)] = {
        "measurement": measurement,
        "topic": topic,
        "log_timestamp": log_timestamp,
    }


def add_sensor_properties(
        properties,
        item,
):
    """
    Add one sensor's data as flat properties.

    Example:
        type = temperature
        id   = 28.5CD456B5013C

    becomes:
        temperature_28_5CD456B5013C_temperature
    """
    measurement = item["measurement"]

    sensor_type = str(
        measurement.get(
            "type",
            "measurement",
        )
    )

    sensor_id = str(
        measurement.get(
            "id",
            "unknown",
        )
    )

    safe_type = sanitize_name(sensor_type)
    safe_id = sanitize_name(sensor_id)
    prefix = f"{safe_type}_{safe_id}"
    data = measurement.get(
        "data",
        {},
    )

    # ---------------------------------------------------------
    # Measurement data
    # ---------------------------------------------------------
    if isinstance(data, dict):
        properties.update(
            flatten_data(
                data,
                prefix,
            )
        )
    else:
        properties[prefix] = data

    # ---------------------------------------------------------
    # Sensor metadata
    # ---------------------------------------------------------
    properties[f"{prefix}_id"] = sensor_id
    properties[f"{prefix}_time"] = measurement.get("time")
    properties[f"{prefix}_topic"] = item["topic"]
    properties[f"{prefix}_log_timestamp"] = item["log_timestamp"]


def create_feature(
        gnss,
        latest_measurements,
):
    """
    Create one GeoJSON feature.

    The GNSS measurement supplies the geometry.

    The latest measurement of every unique sensor
    supplies flat properties.
    """
    measurement = gnss["measurement"]
    data = measurement.get(
        "data",
        {},
    )

    # ---------------------------------------------------------
    # GNSS position
    # ---------------------------------------------------------
    lat = data.get("lat")
    lon = data.get("lon")

    if lat is None or lon is None:
        return None

    # ---------------------------------------------------------
    # GNSS properties
    # ---------------------------------------------------------
    properties = {
        "gnss_lat": lat,
        "gnss_lon": lon,
        "gnss_time": measurement.get("time"),
        "gnss_id": measurement.get("id"),
        "gnss_topic": gnss["topic"],
        "gnss_log_timestamp": gnss["log_timestamp"],
    }

    # Add all GNSS data.
    for key, value in data.items():
        if key in ("lat", "lon"):
            continue
        properties[f"gnss_{sanitize_name(key)}"] = value

    # ---------------------------------------------------------
    # Other sensors
    # ---------------------------------------------------------
    for item in latest_measurements.values():
        add_sensor_properties(
            properties,
            item,
        )

    # ---------------------------------------------------------
    # GeoJSON Feature
    # ---------------------------------------------------------
    return {
        "type": "Feature",
        "geometry": {
            "type": "Point",
            # GeoJSON = [longitude, latitude]
            "coordinates": [
                lon,
                lat,
            ],
        },
        "properties": properties,
    }


def process_file(
        filename,
        features,
        statistics,
):
    """
    Process one log file.

    Every GNSS line produces one feature.

    Between GNSS measurements, only the latest measurement
    for each unique (type, id) sensor is retained.
    """
    current_gnss = None
    latest_measurements = {}
    try:
        with open(filename, "r", encoding="utf-8", ) as f:
            for line_number, line in enumerate(f, start=1, ):
                if not line.strip():
                    continue

                statistics["lines"] += 1

                try:
                    (log_timestamp, topic, measurement,) = parse_line(line)

                except Exception as e:
                    statistics["parse_errors"] += 1
                    print(f"WARNING: {filename}:{line_number}: {e}")
                    continue

                measurement_type = measurement.get("type")

                # -------------------------------------------------
                # GNSS
                # -------------------------------------------------

                if measurement_type == "gnss":
                    statistics["gnss"] += 1

                    # Complete the previous feature.
                    if current_gnss is not None:
                        feature = create_feature(current_gnss, latest_measurements, )

                        if feature is not None:
                            features.append(feature)
                            statistics["features"] += 1
                        else:
                            statistics["missing_position"] += 1

                    # Start a new interval.
                    current_gnss = {
                        "log_timestamp": log_timestamp,
                        "topic": topic,
                        "measurement": measurement,
                    }

                    # Clear all previous sensor values.
                    latest_measurements = {}

                # -------------------------------------------------
                # Other sensor
                # -------------------------------------------------
                else:
                    statistics["other_measurements"] += 1
                    add_latest_measurement(latest_measurements, measurement, topic, log_timestamp)

    except Exception as e:
        print(f"ERROR reading {filename}: {e}")

        statistics["file_errors"] += 1
        return

    # ---------------------------------------------------------
    # Final feature
    # ---------------------------------------------------------
    if current_gnss is not None:
        feature = create_feature(
            current_gnss,
            latest_measurements,
        )

        if feature is not None:
            features.append(feature)
            statistics["features"] += 1
        else:
            statistics["missing_position"] += 1


def find_files(input_directory, recursive=False, ):
    """
    Find all files in the input directory.
    """
    if recursive:
        files = [path for path in input_directory.rglob("*") if path.is_file()]
    else:
        files = [path for path in input_directory.iterdir() if path.is_file()]

    # Filename ordering.
    files.sort()

    return files


def main():
    parser = argparse.ArgumentParser(
        description=("Convert GNSS/sensor logs to GeoJSON.")
    )

    parser.add_argument(
        "input_directory",
        type=Path,
        help="Directory containing log files",
    )

    parser.add_argument(
        "output",
        type=Path,
        help="Output GeoJSON file",
    )

    parser.add_argument(
        "--recursive",
        action="store_true",
        help=("Search for files recursively in subdirectories"),
    )

    args = parser.parse_args()

    # ---------------------------------------------------------
    # Validate input
    # ---------------------------------------------------------
    if not args.input_directory.exists():
        raise SystemExit(f"Directory does not exist: {args.input_directory}")

    if not args.input_directory.is_dir():
        raise SystemExit(f"Not a directory: {args.input_directory}")

    # ---------------------------------------------------------
    # Find files
    # ---------------------------------------------------------
    files = find_files(
        args.input_directory,
        args.recursive,
    )

    if not files:
        raise SystemExit("No input files found.")

    print(f"Found {len(files)} files.")

    # ---------------------------------------------------------
    # Statistics
    # ---------------------------------------------------------
    statistics = {
        "lines": 0,
        "gnss": 0,
        "other_measurements": 0,
        "features": 0,
        "parse_errors": 0,
        "missing_position": 0,
        "file_errors": 0,
    }

    # ---------------------------------------------------------
    # Process
    # ---------------------------------------------------------
    features = []

    for index, filename in enumerate(
            files,
            start=1,
    ):
        print(f"[{index}/{len(files)}] {filename}")

        process_file(
            filename,
            features,
            statistics,
        )

    # ---------------------------------------------------------
    # Create GeoJSON
    # ---------------------------------------------------------
    geojson = {
        "type": "FeatureCollection",
        "features": features,
    }

    args.output.parent.mkdir(parents=True, exist_ok=True, )

    with open(args.output, "w", encoding="utf-8", ) as f:
        json.dump(
            geojson,
            f,
            indent=2,
            ensure_ascii=False,
        )

    # ---------------------------------------------------------
    # Summary
    # ---------------------------------------------------------
    print()
    print("Conversion complete")
    print("--------------------")
    print(f"Files:              {len(files)}")
    print(f"Lines:              {statistics['lines']}")
    print(f"GNSS measurements:  {statistics['gnss']}")
    print(f"Other measurements: {statistics['other_measurements']}")
    print(f"GeoJSON features:   {statistics['features']}")
    print(f"Parse errors:       {statistics['parse_errors']}")
    print(f"Missing positions:  {statistics['missing_position']}")
    print(f"File errors:        {statistics['file_errors']}")
    print()
    print(f"Output: {args.output}")


if __name__ == "__main__":
    main()
