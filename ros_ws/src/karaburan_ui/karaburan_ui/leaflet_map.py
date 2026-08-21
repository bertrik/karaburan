from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


EARTH_RADIUS_METERS = 6378137.0


def map_point_to_wgs84(x, y, origin_latitude, origin_longitude):
    """Convert local ENU map coordinates to WGS84 for short routes."""
    latitude = origin_latitude + math.degrees(y / EARTH_RADIUS_METERS)
    longitude = origin_longitude + math.degrees(
        x / (EARTH_RADIUS_METERS * math.cos(math.radians(origin_latitude)))
    )
    return [latitude, longitude]


class MapState:

    def __init__(self, origin_latitude, origin_longitude):
        self._lock = threading.Lock()
        self._state = {
            'origin': [origin_latitude, origin_longitude],
            'position': None,
            'track': [],
            'plan': [],
        }
        self._origin_latitude = origin_latitude
        self._origin_longitude = origin_longitude

    def update_fix(self, message):
        if not math.isfinite(message.latitude):
            return
        if not math.isfinite(message.longitude):
            return
        point = [message.latitude, message.longitude]
        with self._lock:
            self._state['position'] = point
            track = self._state['track']
            if not track or point != track[-1]:
                track.append(point)
                del track[:-10000]

    def update_plan(self, message):
        plan = [
            map_point_to_wgs84(
                pose.pose.position.x,
                pose.pose.position.y,
                self._origin_latitude,
                self._origin_longitude,
            )
            for pose in message.poses
        ]
        with self._lock:
            self._state['plan'] = plan

    def to_json(self):
        with self._lock:
            return json.dumps(self._state).encode('utf-8')


def make_handler(state, html):
    class MapRequestHandler(BaseHTTPRequestHandler):

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self._reply(200, 'text/html; charset=utf-8', html)
            elif self.path == '/state':
                self._reply(200, 'application/json', state.to_json())
            elif self.path == '/healthz':
                self._reply(200, 'text/plain; charset=utf-8', b'ok\n')
            else:
                self._reply(404, 'text/plain; charset=utf-8', b'not found\n')

        def log_message(self, _format, *_args):
            pass

        def _reply(self, status, content_type, body):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    return MapRequestHandler


class LeafletMap(Node):

    def __init__(self):
        super().__init__('leaflet_map')
        self.declare_parameter('bind_address', '0.0.0.0')
        self.declare_parameter('port', 8088)
        self.declare_parameter('origin_latitude', 52.018599)
        self.declare_parameter('origin_longitude', 4.708720)
        bind_address = self.get_parameter('bind_address').value
        port = self.get_parameter('port').value
        origin_latitude = self.get_parameter('origin_latitude').value
        origin_longitude = self.get_parameter('origin_longitude').value

        state = MapState(origin_latitude, origin_longitude)
        html_path = Path(get_package_share_directory('karaburan_ui')) / (
            'web/leaflet_map.html'
        )
        html = html_path.read_bytes()
        self._server = ThreadingHTTPServer(
            (bind_address, port), make_handler(state, html)
        )
        self._server_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True
        )
        self._server_thread.start()
        self.create_subscription(
            NavSatFix, '/fix/valid', state.update_fix, 10
        )
        self.create_subscription(NavPath, '/plan', state.update_plan, 10)
        self.get_logger().info(
            'Leaflet navigation map available at http://%s:%d'
            % (bind_address, port)
        )

    def destroy_node(self):
        self._server.shutdown()
        self._server.server_close()
        self._server_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LeafletMap()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
