"""One image slot; bounded pose history exists only to align that newest image."""
from collections import deque
import threading
import time


class LatestObservation:
    def __init__(self, slop=0.05, pose_count=120):
        self.lock = threading.Lock()
        self.image = None
        self.poses = {'odin': deque(maxlen=pose_count), 'mavros': deque(maxlen=pose_count)}
        self.slop = slop
        self.overwritten = 0

    def put_image(self, image):
        with self.lock:
            if self.image is not None:
                self.overwritten += 1
            self.image = (image, time.monotonic())

    def put_pose(self, name, message):
        with self.lock:
            self.poses[name].append(message)

    def snapshot(self, ros_now, max_age=0.30):
        with self.lock:
            if self.image is None:
                return None
            image, received = self.image
            stamp = image.header.stamp.to_sec()
            if not 0 <= ros_now - stamp <= max_age or time.monotonic() - received > max_age:
                return None
            paired = []
            for name in ('odin', 'mavros'):
                if not self.poses[name]:
                    return None
                pose = min(self.poses[name], key=lambda p: abs(p.header.stamp.to_sec() - stamp))
                if abs(pose.header.stamp.to_sec() - stamp) > self.slop:
                    return None  # Never fall back to an older, easier-to-pair image.
                paired.append(pose)
            return image, paired[0], paired[1]
