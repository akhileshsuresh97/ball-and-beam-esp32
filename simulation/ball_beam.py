"""Ball-and-beam simulation: physics, sensor, stepper actuator and two controllers.

The physical values follow the project hardware: a 48 cm beam, a table-tennis
ball (radius 20 mm, 2.7 g), a 1.8° stepper motor with a microstep driver, and a
time-of-flight sensor at one end of the beam measuring the distance to the ball.

Two controllers are modelled:
- OriginalController reproduces the logic of the sketch from the presentations.
- ImprovedController is the logic of firmware/ball_on_beam/ball_on_beam.ino.
"""

from dataclasses import dataclass, field
import math

import numpy as np

G = 9.81  # m/s²


@dataclass
class Hardware:
    beam_length: float = 0.48  # m
    ball_radius: float = 0.020  # m (table-tennis ball, thin hollow shell)
    max_beam_angle: float = math.radians(15)  # mechanical stop of the linkage
    steps_per_rev: int = 200 * 8  # 1.8° motor at 1/8 microstepping
    gear_ratio: float = 1.0  # beam angle per motor angle (linkage approximated as 1:1)
    max_step_rate: float = 4000.0  # steps/s the motor can follow without stalling
    max_step_accel: float = 40000.0  # steps/s², speed ramp as in the firmware (AccelStepper)
    sensor_noise: float = 0.003  # m, standard deviation of the ToF reading
    sensor_period: float = 0.020  # s, ToF measurement time in high-speed mode
    friction: float = 0.05  # 1/s, rolling resistance as linear damping

    @property
    def rad_per_step(self) -> float:
        return 2 * math.pi / self.steps_per_rev * self.gear_ratio

    @property
    def accel_per_rad(self) -> float:
        """Ball acceleration per radian of tilt (small angle).

        A rolling hollow sphere has J = 2/3·m·r², so a = g·sinθ / (1 + J/(m r²)) = 3/5·g·sinθ.
        """
        return 0.6 * G


@dataclass
class State:
    x: float  # ball position from the sensor end, m
    v: float = 0.0  # ball velocity, m/s
    steps: int = 0  # motor position in steps (0 = level beam)

    def beam_angle(self, hw: Hardware) -> float:
        return float(np.clip(self.steps * hw.rad_per_step, -hw.max_beam_angle, hw.max_beam_angle))


def step_ball(state: State, hw: Hardware, dt: float) -> None:
    """Advance the ball by dt. Positive beam angle rolls the ball away from the sensor."""
    theta = state.beam_angle(hw)
    a = hw.accel_per_rad * math.sin(theta) - hw.friction * state.v
    state.v += a * dt
    state.x += state.v * dt
    lo, hi = hw.ball_radius, hw.beam_length - hw.ball_radius
    if state.x < lo or state.x > hi:  # end stops: the ball hits the end and stops
        state.x = min(max(state.x, lo), hi)
        state.v = 0.0


class OriginalController:
    """The sketch from the presentations, translated faithfully.

    - Every 50 ms: error = setpoint − distance (mm); integral and derivative use
      milliseconds as the time unit; output = Kp·e + Ki·∫e + Kd·de/dt.
    - controlMotor(output) then moves |output| *additional* steps in the direction
      of the sign, blocking for 1 ms per step (two 500 µs delays) while nothing
      else runs, including sensor reads.
    """

    def __init__(self, kp=1.2, ki=0.5, kd=0.1, setpoint_mm=100.0, sample_ms=50):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint_mm
        self.sample = sample_ms / 1000
        self.integral = 0.0
        self.last_error = 0.0
        self.busy_until = 0.0
        self.next_sample = 0.0
        self.pending_steps = 0  # steps still to be pulsed by the blocking loop
        self.direction = 1

    def update(self, t: float, measured_mm: float, state: State, hw: Hardware, dt: float) -> None:
        if self.pending_steps > 0:  # still inside the blocking for-loop
            n = min(self.pending_steps, max(1, round(dt / 0.001)))  # 1 ms per step
            state.steps += self.direction * n
            self.pending_steps -= n
            if self.pending_steps == 0:
                self.next_sample = max(self.next_sample, t)
            return
        if t < self.next_sample:
            return
        elapsed_ms = self.sample * 1000
        error = self.setpoint - measured_mm
        self.integral += error * elapsed_ms
        derivative = (error - self.last_error) / elapsed_ms
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        self.next_sample = t + self.sample
        self.pending_steps = int(abs(output))
        self.direction = 1 if output > 0 else -1


class ImprovedController:
    """The logic of the improved firmware.

    - Moving-average filter on the distance, in metres.
    - PID with time in seconds, derivative on the measurement (no kick on
      setpoint changes) and a clamped integral (anti-windup).
    - The PID output is a target beam *angle*, clamped to the mechanical limits.
    - The motor moves to that absolute angle, at most max_step_rate steps/s,
      without blocking: sensor reads and control continue while it moves.
    """

    def __init__(self, kp=2.2, ki=0.2, kd=0.6, setpoint_m=0.24, sample_s=0.020,
                 filter_len=8, max_angle_deg=12.0, integral_limit=0.05):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint_m
        self.sample = sample_s
        self.window: list[float] = []
        self.filter_len = filter_len
        self.max_angle = math.radians(max_angle_deg)
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.last_x: float | None = None
        self.next_sample = 0.0
        self.target_steps = 0
        self.step_accum = 0.0
        self.speed = 0.0  # steps/s

    def update(self, t: float, measured_mm: float, state: State, hw: Hardware, dt: float) -> None:
        if t >= self.next_sample:
            self.next_sample = t + self.sample
            self.window = (self.window + [measured_mm / 1000])[-self.filter_len:]
            x = sum(self.window) / len(self.window)
            error = x - self.setpoint  # ball too far from the sensor → tilt it back
            self.integral = float(np.clip(self.integral + error * self.sample,
                                          -self.integral_limit, self.integral_limit))
            dx = 0.0 if self.last_x is None else (x - self.last_x) / self.sample
            self.last_x = x
            angle = -(self.kp * error + self.ki * self.integral + self.kd * dx)
            angle = float(np.clip(angle, -self.max_angle, self.max_angle))
            self.target_steps = round(angle / hw.rad_per_step)
        # Non-blocking stepping towards the target, with the speed and acceleration
        # limits of AccelStepper: accelerate, cruise, and brake in time to stop on target.
        diff = self.target_steps - state.steps
        brake_speed = math.sqrt(2 * hw.max_step_accel * abs(diff))
        desired = math.copysign(min(hw.max_step_rate, brake_speed), diff) if diff else 0.0
        dv = float(np.clip(desired - self.speed, -hw.max_step_accel * dt, hw.max_step_accel * dt))
        self.speed += dv
        self.step_accum += self.speed * dt
        n = int(self.step_accum)  # truncates towards zero for both directions
        self.step_accum -= n
        state.steps += n


@dataclass
class Result:
    t: np.ndarray
    x: np.ndarray
    angle_deg: np.ndarray
    setpoint: float
    hw: Hardware = field(repr=False)

    def settled(self, tol: float = 0.02, hold: float = 1.0) -> float | None:
        """First time after which the ball stays within ±tol of the setpoint for `hold` seconds
        and until the end. Returns None if it never settles."""
        inside = np.abs(self.x - self.setpoint) <= tol
        if not inside[-1]:
            return None
        last_out = np.where(~inside)[0]
        t_in = self.t[last_out[-1] + 1] if len(last_out) else self.t[0]
        return float(t_in) if self.t[-1] - t_in >= hold else None

    def rms_error(self, after: float = 0.0) -> float:
        m = self.t >= after
        return float(np.sqrt(np.mean((self.x[m] - self.setpoint) ** 2)))

    def end_hits(self) -> int:
        """How often the ball struck an end of the beam."""
        lo, hi = self.hw.ball_radius, self.hw.beam_length - self.hw.ball_radius
        at_end = (self.x <= lo + 1e-6) | (self.x >= hi - 1e-6)
        return int(np.sum(at_end[1:] & ~at_end[:-1]))


def simulate(controller, x0: float = 0.08, duration: float = 10.0, dt: float = 0.001,
             hw: Hardware | None = None, seed: int = 0,
             disturbance: tuple[float, float] | None = None) -> Result:
    """Run a closed-loop simulation.

    x0: start position of the ball from the sensor end (m).
    disturbance: optional (time, velocity) push given to the ball, in m/s.
    """
    hw = hw or Hardware()
    rng = np.random.default_rng(seed)
    state = State(x=x0)
    n = int(duration / dt)
    ts, xs, angles = np.empty(n), np.empty(n), np.empty(n)
    reading = state.x * 1000
    next_reading = 0.0
    pushed = False
    for i in range(n):
        t = i * dt
        if disturbance and not pushed and t >= disturbance[0]:
            state.v += disturbance[1]
            pushed = True
        if t >= next_reading:  # the sensor delivers a new value every sensor_period
            reading = (state.x + rng.normal(0, hw.sensor_noise)) * 1000
            next_reading = t + hw.sensor_period
        controller.update(t, reading, state, hw, dt)
        step_ball(state, hw, dt)
        ts[i], xs[i], angles[i] = t, state.x, math.degrees(state.beam_angle(hw))
    setpoint = controller.setpoint / 1000 if isinstance(controller, OriginalController) else controller.setpoint
    return Result(ts, xs, angles, setpoint, hw)
