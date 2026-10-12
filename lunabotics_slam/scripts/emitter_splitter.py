#!/usr/bin/env python3
"""Split an emitter on/off interleaved D455 stream by projector state.

With depth_module.emitter_on_off the IR projector is on every other frame. Each
frame's metadata (frame_emitter_mode) says which, and IR/depth of the same frameset
share a timestamp. This node routes:
  projector OFF -> /split/infra1, /split/infra2  (clean IR for stereo visual odometry)
  projector ON  -> /split/depth                  (dense depth for the point cloud)
Frames are matched to metadata by timestamp, so dropped frames don't break the split.

Images are forwarded as raw serialized bytes (never deserialized), so Python keeps
up with three 30 Hz image streams. Each routed image is published right after its
camera_info, restamped in place, so downstream exact-time sync never misses a pair.
"""
import collections
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from realsense2_camera_msgs.msg import Metadata
from sensor_msgs.msg import CameraInfo, Image

# stream -> projector state it is kept for (0 = off, 1 = on)
STREAMS = {'infra1': 0, 'infra2': 0, 'depth': 1}
MAX_PENDING = 60
# Reliable, matching the driver: best-effort loses most ~800 KB depth images to
# dropped DDS fragments.
QOS = QoSProfile(depth=5)


def raw_stamp(data):
    # CDR: 4-byte encapsulation header, then std_msgs/Header.stamp (int32 sec, uint32 nanosec).
    sec, nsec = struct.unpack_from('<iI' if data[1] == 1 else '>iI', data, 4)
    return sec * 1_000_000_000 + nsec


def emitter_mode(json_data):
    key = '"frame_emitter_mode":'
    i = json_data.find(key)
    return None if i < 0 else int(json_data[i + len(key)])


class EmitterSplitter(Node):
    def __init__(self):
        super().__init__('emitter_splitter')
        self.modes = collections.OrderedDict()
        self.pending = collections.OrderedDict()
        self.infos = {}
        self.image_pubs, self.info_pubs = {}, {}
        for stream in STREAMS:
            self.image_pubs[stream] = self.create_publisher(Image, f'/split/{stream}/image_rect_raw', 5)
            self.info_pubs[stream] = self.create_publisher(CameraInfo, f'/split/{stream}/camera_info', 5)
            self.create_subscription(Image, f'/camera/{stream}/image_rect_raw',
                                     lambda data, s=stream: self.on_image(s, data), QOS, raw=True)
            self.create_subscription(CameraInfo, f'/camera/{stream}/camera_info',
                                     lambda data, s=stream: self.infos.__setitem__(s, data),
                                     QOS, raw=True)
        # Depth metadata carries the frameset's emitter state (same stamp as IR).
        self.create_subscription(Metadata, '/camera/depth/metadata', self.on_metadata, QOS)

    def on_metadata(self, msg):
        mode = emitter_mode(msg.json_data)
        if mode is None:
            self.get_logger().warn('metadata has no frame_emitter_mode; is emitter_on_off set?',
                                   throttle_duration_sec=5.0)
            return
        t = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.modes[t] = mode
        while len(self.modes) > MAX_PENDING:
            self.modes.popitem(last=False)
        for stream in STREAMS:
            data = self.pending.pop((stream, t), None)
            if data is not None:
                self.route(stream, data, mode)

    def on_image(self, stream, data):
        t = raw_stamp(data)
        mode = self.modes.get(t)
        if mode is None:  # metadata not here yet
            self.pending[(stream, t)] = data
            while len(self.pending) > MAX_PENDING:
                self.pending.popitem(last=False)
            return
        self.route(stream, data, mode)

    def route(self, stream, data, mode):
        info = self.infos.get(stream)
        if mode != STREAMS[stream] or info is None:
            return
        # Calibration is constant; copy the image's stamp bytes into the latest info.
        self.info_pubs[stream].publish(info[:4] + data[4:12] + info[12:])
        self.image_pubs[stream].publish(data)


def main():
    rclpy.init()
    node = EmitterSplitter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl-C / ros2 launch stop
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
