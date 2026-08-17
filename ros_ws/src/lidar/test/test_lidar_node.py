from types import SimpleNamespace

from lidar.lidar_node import LidarNode, MysteryLidar


class _Time:

    def to_msg(self):
        return 'measurement-time'


class _Clock:

    def now(self):
        return _Time()


class _Publisher:

    def __init__(self):
        self.stamps = []

    def publish(self, msg):
        self.stamps.append(msg.header.stamp)


def _encoded_angle(angle):
    return round((angle + 640.0) * 64.0)


def test_completed_scan_is_stamped_before_publish():
    frame = bytearray(60)
    first_angle = _encoded_angle(350.0)
    last_angle = _encoded_angle(10.0)
    frame[6:8] = first_angle.to_bytes(2, 'little')
    frame[56:58] = last_angle.to_bytes(2, 'little')

    node = SimpleNamespace(
        get_clock=lambda: _Clock(),
        msg=SimpleNamespace(
            header=SimpleNamespace(stamp=None),
            ranges=[0.0] * MysteryLidar.RAYS_PER_ROTATION,
            intensities=[0.0] * MysteryLidar.RAYS_PER_ROTATION,
        ),
        publisher=_Publisher(),
    )

    LidarNode.process_frame(node, bytes(frame))

    assert node.publisher.stamps == ['measurement-time']
