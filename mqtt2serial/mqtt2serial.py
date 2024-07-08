#!/usr/bin/env python3

""" receives commands over mqtt, forwards them to serial, sends the serial response back to mqtt """

import argparse
import json

import paho.mqtt.client as mqtt
import serial


class Responder:
    """ mqtt responder """
    def __init__(self, ser, broker, base_topic, config):
        self.serial = ser
        self.broker = broker
        self.config = config

        self.config_topic = f"{base_topic}/config"
        self.command_topic = f"{base_topic}/command"
        self.response_topic = f"{base_topic}/response"

        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.will_set(self.config_topic, None, retain=True)

    def start(self):
        """" starts the responder, connects to mqtt """
        print(f"Connecting to MQTT broker {self.broker}...")
        self.client.connect(self.broker)
        self.client.loop_start()

    def stop(self):
        """ stops the responder, disconnects from mqtt """
        self.client.disconnect()

    def on_connect(self, _client, _userdata, _flags, _rc):
        """ handles mqtt connect event: publish our config and subscribe to commands """
        print(f"Connected, subscribing to topic {self.command_topic}...")
        self.client.publish(self.config_topic, json.dumps(self.config), retain=True)
        self.client.subscribe(self.command_topic)

    def on_message(self, _client, _userdata, msg):
        """ sends a command message from mqtt to serial """
        command = msg.payload.decode('UTF-8')
        print(f">>>MQTT>>>serial: '{command}'")
        self.serial.write(msg.payload)
        self.serial.write(b'\n')

    def send_response(self, data):
        """ sends a response message from serial to mqtt """
        response = data.decode('UTF-8')
        print(f" <<MQTT<<<serial: '{response}'")
        self.client.publish(self.response_topic, data)


def main():
    """ The main entry point """
    parser = argparse.ArgumentParser()
    parser.add_argument("--broker", help="The MQTT broker host name", default="localhost")
    parser.add_argument("--type", help="The device type", default="motor")
    parser.add_argument("--id", help="The device id", default="1")
    parser.add_argument("--serial", help="The serial device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", help="The serial port baud rate", default="115200")
    args = parser.parse_args()

    # set up topic structure
    base_topic = f"karaburan/control/{args.type}/{args.id}"
    config = {'type': args.type, 'id': args.id}

    # Open the serial port
    print(f"Opening {args.serial} @ {args.baud} bps...")
    with serial.Serial(args.serial, args.baud) as ser:
        responder = Responder(ser, args.broker, base_topic, config)
        responder.start()
        try:
            while True:
                data = ser.readline()
                if data:
                    responder.send_response(data.strip())
        except serial.serialutil.SerialException:
            responder.stop()


if __name__ == "__main__":
    main()
