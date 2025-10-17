#!/usr/bin/env python3
# Copyright 2025 Bertrik Sikken <bertrik@sikken.nl>
import asyncio
import math
import queue
import threading
from dataclasses import dataclass, field
from enum import IntFlag

import rclpy
from bleak import BleakClient, BleakError
from rclpy.node import Node
from sensor_msgs.msg import Range


class BleSerialPort:
    NOTIFY_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"
    WRITE_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"
    READ_UUID = "0000fff3-0000-1000-8000-00805f9b34fb"

    def __init__(self, address, reconnect_delay=3):
        self.address = address
        self.reconnect_delay = reconnect_delay
        self.loop = asyncio.new_event_loop()
        self.client: BleakClient | None = None
        self.rx_queue: queue.Queue[int] = queue.Queue()
        self._closing = False
        self._thread = threading.Thread(target=self._run_loop, daemon=True)

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def open(self):
        self._thread.start()
        self._call(self._connect())  # block until connected

    async def _connect(self):
        while not self._closing:
            try:
                print("Connecting...")
                self.client = BleakClient(self.address, disconnected_callback=self._on_disconnect)
                await self.client.connect()
                await self.client.start_notify(self.NOTIFY_UUID, self._on_notify)
                print(f"Connected to {self.address}")
                return
            except Exception as e:
                print(f"Connection failed: {e}")
                await asyncio.sleep(self.reconnect_delay)

    def _on_disconnect(self, _client):
        if not self._closing:
            print("Disconnected, will retry...")
            asyncio.run_coroutine_threadsafe(self._connect(), self.loop)

    def _on_notify(self, _sender, data: bytearray):
        for b in data:
            self.rx_queue.put(b)

    def write(self, data: bytes, timeout=None):
        if not self.client or not self.client.is_connected:
            raise BleakError("Not connected")
        return self._call(self.client.write_gatt_char(self.WRITE_UUID, data), timeout)

    def read(self, timeout=None) -> int | None:
        try:
            return self.rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self):
        self._closing = True
        self._call(self._disconnect())
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join()
        self.loop.close()

    async def _disconnect(self):
        if self.client and self.client.is_connected:
            await self.client.stop_notify(self.NOTIFY_UUID)
            await self.client.disconnect()
        self.client = None
        print("Disconnected cleanly")

    def _call(self, coro, timeout=None):
        fut = asyncio.run_coroutine_threadsafe(coro, self.loop)
        return fut.result(timeout=timeout)

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class Protocol:
    NOISE_OFF = 0
    NOISE_LOW = 1
    NOISE_MEDIUM = 2
    NOISE_HIGH = 3

    RANGE_AUTO = 0
    RANGE_10FT = 1
    RANGE_20FT = 2
    RANGE_30FT = 3
    RANGE_60FT = 4
    RANGE_90FT = 5
    RANGE_120FT = 6
    RANGE_150FT = 7
    RANGE_200FT = 8

    def __init__(self):
        self.index = 0
        self.buffer = bytearray(140)

    def build_configcmd(self, noise_filter, sensitivity, depth_range) -> bytes:
        command = bytearray(10)
        command[0] = 0x53  # 'S'
        command[1] = 0x46  # 'F'
        command[2] = 1
        command[3] = noise_filter  # 0..3
        command[4] = sensitivity  # percent
        command[5] = 0
        command[6] = depth_range
        command[7] = 0
        command[8] = sum(command[0:8]) & 0xFF
        command[9] = 0x55  # 'U'
        return command

    def process(self, b) -> bytes | None:
        # store byte if it fits
        if self.index < len(self.buffer):
            self.buffer[self.index] = b
            # print(f" {b:02X}", end='')

        # check specific markers
        match self.index:
            case 0:
                if b != 0x53:  # 'S'
                    self.index = 0
                    return None
            case 1:
                if b != 0x46:  # 'F'
                    self.index = 0
                    return self.process(b)
            case 13:
                # calculate checksum
                check = sum(self.buffer[0:13]) & 0xFF
                if check != b:
                    self.index = 0
                    return self.process(b)
            case 14 | 136 | 138:
                if b != 0x55:
                    self.index = 0
                    return self.process(b)
            case 135 | 137 | 139:
                if b != 0xAA:
                    self.index = 0
                    return self.process(b)

        # next byte or done?
        self.index += 1
        if self.index == 140:
            self.index = 0
            return bytes(self.buffer)

        return None


@dataclass
class SensorData:
    status: int
    bottom: int
    fishdepth: int
    fishsize: int
    battery: int
    temperature: int
    frequency: int
    depthrange: int
    rawdata: bytes = field(repr=False)

    class Status(IntFlag):
        CHARGING = 0x80
        CHARGE_DONE = 0x40
        OUT_OF_WATER = 0x08

    @classmethod
    def from_bytes(cls, data: bytes):
        # Expect at least 13 bytes (since we access up to index 12)
        if len(data) < 13:
            raise ValueError(f"SensorData requires at least 13 bytes, got {len(data)}")
        return cls(
            status=data[2],
            bottom=(data[3] << 8) + data[4],
            fishdepth=(data[5] << 8) + data[6],
            fishsize=data[7],
            battery=data[8],
            temperature=(data[9] << 8) + data[10],
            frequency=data[11],
            depthrange=data[12],
            rawdata=data[15:135]
        )

    @staticmethod
    def _ft_to_meter(depth):
        return depth * 0.3048

    # status
    def get_status(self) -> set:
        return set(self.Status(self.status))

    # depth in meters
    def get_depth(self) -> float:
        return self._ft_to_meter(self.bottom / 10.0)

    # battery in percentage
    def get_battery(self) -> float:
        return self.battery * 100.0 / 6

    # temperature in degrees Celcius
    def get_temperature(self) -> float:
        return (self.temperature / 10.0 - 32) * 5 / 9 if self.temperature > 0 else math.nan

    # depth range in meters
    def get_depth_range(self) -> float:
        return self._ft_to_meter(self.depthrange)


class SonarNode(Node):
    def __init__(self):
        super().__init__('sonar')
        self.declare_parameter('device', value='D3:01:01:02:2F:C6')
        self.declare_parameter('topic', value='/sonar')
        self.declare_parameter('frame_id', value='sonar')

        self.device = self.get_parameter('device').get_parameter_value().string_value
        # self.sonar = Sonar(device)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self.publisher = self.create_publisher(Range, topic, 10)

        # initialise Range message
        self.msg = Range()
        self.msg.header.frame_id = self.frame_id
        self.msg.radiation_type = Range.ULTRASOUND
        self.msg.field_of_view = math.radians(45)
        self.msg.min_range = 0.3  # estimated
        self.msg.max_range = 60.0  # 200 feet actually

        # create and start listen thread
        self.thread = threading.Thread(target=self.node_task())
        self.thread.start()

    def node_task(self) -> None:
        protocol = Protocol()
        with BleSerialPort(self.device) as port:
            while rclpy.ok():
                while True:
                    b = port.read()
                    frame = protocol.process(b)
                    if frame:
                        sd = SensorData.from_bytes(frame)
                        if sd and not sd.get_status():
                            self.publish_range(sd)
                        print(f"status={sd.get_status()},temp={sd.get_temperature():.1f}degC,"
                              f"batt={sd.get_battery():.1f}%,"
                              f"depth={sd.get_depth():.2f}m,range={sd.get_depth_range():.1f}m")

    def publish_range(self, sd: SensorData) -> None:
        now = self.get_clock().now()

        # publish message
        self.msg.header.stamp = now.to_msg()
        self.msg.max_range = sd.get_depth_range()
        self.msg.range = sd.get_depth()
        self.publisher.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    sonar_node = SonarNode()
    rclpy.spin(sonar_node)
    sonar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
