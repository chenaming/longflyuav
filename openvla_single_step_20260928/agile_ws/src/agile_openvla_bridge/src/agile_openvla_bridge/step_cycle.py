"""One request and one goal per trigger, with continuous arrival dwell."""
import math


class StepCycle:
    def __init__(self, distance=0.04, speed=0.10, dwell=1.0):
        if any(not math.isfinite(v) or v <= 0 for v in (distance, speed, dwell)):
            raise ValueError("arrival limits must be positive and finite")
        self.distance, self.speed, self.dwell = distance, speed, dwell
        self.cancel()

    def cancel(self):
        self.pending = self.inflight = False
        self.settled_since = None

    def request(self, executing):
        if executing or self.pending or self.inflight:
            raise ValueError("step already queued, inferring or executing")
        self.pending = True

    def begin(self):
        if not self.pending:
            return False
        self.pending, self.inflight = False, True
        return True

    def arrived(self, distance, velocity, now):
        if (not math.isfinite(distance) or not math.isfinite(velocity)
                or distance > self.distance or velocity > self.speed):
            self.settled_since = None
            return False
        if self.settled_since is None:
            self.settled_since = now
        return now - self.settled_since >= self.dwell
