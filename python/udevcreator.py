#!/usr/bin/env python3

# Copyright 2025 Bertrik Sikken <bertrik@sikken.nl>

import pyudev


def main():
    # Create udev context and monitor
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by(subsystem='tty')  # Only listen for tty devices (like /dev/ttyUSB0)

    # Monitor events indefinitely
    print("Monitoring for USB-Serial device insertions/removals...")
    for device in iter(monitor.poll, None):
        properties = device.properties

        # print(f"All properties: {properties}")
        # for p in properties:
        #     val = properties.get(p)
        #     print(f'{p}={val}')

        items = []
        if 'ID_BUS' in properties and properties['ID_BUS'] == 'usb':
            vendor_string = properties["ID_VENDOR_FROM_DATABASE"]
            model_string = properties["ID_MODEL_FROM_DATABASE"]
            model_enc = properties["ID_USB_MODEL_ENC"]

            print(f"# {vendor_string} / {model_string} ({model_enc})")
            items.append('SUBSYSTEM=="tty"')
            vendor_id = properties['ID_USB_VENDOR_ID']
            items.append(f'ATTRS{{idVendor}}=="{vendor_id}"')
            product_id = properties['ID_USB_MODEL_ID']
            items.append(f'ATTRS{{idProduct}}=="{product_id}"')
            items.append('MODE="0660"')
            items.append('GROUP="dialout"')
            items.append(f'SYMLINK+="tty{product_id}"')
            udev = ", ".join(items)
            print(f'{udev}')


if __name__ == "__main__":
    main()
