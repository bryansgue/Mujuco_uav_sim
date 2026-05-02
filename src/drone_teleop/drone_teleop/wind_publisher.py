#!/usr/bin/env python3
"""Realistic wind/disturbance publisher for the MuJoCo quadrotor.

Publishes a persistent external force to /<quad_name>/external_force combining
three components that are standard in aerospace disturbance modelling:

  1. Mean wind       — Ornstein-Uhlenbeck process (slowly varying baseline).
  2. Turbulence      — first-order Dryden-like filter (MIL-HDBK-1797 inspired).
  3. Discrete gusts  — random impulses with (1-cos) envelope.

Resulting force = drag_coef * rho * A * v_wind * |v_wind|  +  gust
(simple quadratic drag model; good enough for control-robustness experiments).

The ExternalForce MuJoCo plugin must be compiled with persistent=true so that
each new message overrides the previous force without waiting for a duration.
"""

import math
import random

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Vector3
from mujoco_ros_utils.msg import ExternalForce
from rclpy.node import Node


TURBULENCE_LEVELS = {
    # sigma_u [m/s], L_u [m]   (escalado para UAV pequeño ~1 kg)
    "light":    (0.3, 50.0),
    "moderate": (0.8, 50.0),
    "severe":   (1.5, 50.0),
}


class OrnsteinUhlenbeck:
    """Scalar OU process: dx = -x/tau dt + sigma sqrt(2/tau) dW."""

    def __init__(self, tau: float, sigma: float, x0: float = 0.0):
        self.tau = tau
        self.sigma = sigma
        self.x = x0

    def step(self, dt: float) -> float:
        a = math.exp(-dt / self.tau)
        noise = self.sigma * math.sqrt(1.0 - a * a) * random.gauss(0.0, 1.0)
        self.x = a * self.x + noise
        return self.x


class DrydenTurbulence:
    """First-order approximation of Dryden longitudinal spectrum.

    H(s) = sigma * sqrt(2 L / (pi V)) / (1 + (L/V) s)
    Discretised as a first-order low-pass of white noise.
    """

    def __init__(self, sigma: float, L: float, V: float = 10.0):
        self.sigma = sigma
        self.L = L
        self.V = max(V, 1.0)
        self.state = np.zeros(3)

    def step(self, dt: float) -> np.ndarray:
        tau = self.L / self.V
        a = math.exp(-dt / tau)
        gain = self.sigma * math.sqrt(1.0 - a * a)
        w = np.array([random.gauss(0.0, 1.0) for _ in range(3)])
        self.state = a * self.state + gain * w
        return self.state.copy()


class Gust:
    """Discrete (1-cos) gust in a random direction."""

    def __init__(self):
        self.remaining = 0.0
        self.duration = 0.0
        self.amplitude = 0.0
        self.direction = np.zeros(3)

    def trigger(self, amplitude: float, duration: float):
        self.amplitude = amplitude
        self.duration = duration
        self.remaining = duration
        theta = random.uniform(0.0, 2.0 * math.pi)
        phi = random.uniform(-0.3, 0.3)  # mostly horizontal
        self.direction = np.array(
            [math.cos(theta) * math.cos(phi), math.sin(theta) * math.cos(phi), math.sin(phi)]
        )

    def step(self, dt: float) -> np.ndarray:
        if self.remaining <= 0.0:
            return np.zeros(3)
        t = self.duration - self.remaining
        env = 0.5 * (1.0 - math.cos(2.0 * math.pi * t / self.duration))
        self.remaining -= dt
        return self.amplitude * env * self.direction


class WindPublisher(Node):
    def __init__(self):
        super().__init__("wind_publisher")

        self.declare_parameter("quad_name", "quadrotor")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("turbulence_level", "moderate")
        self.declare_parameter("mean_wind_speed", 1.5)      # m/s
        self.declare_parameter("mean_wind_tau", 10.0)       # s (OU time constant)
        self.declare_parameter("mean_wind_sigma", 0.4)      # m/s
        self.declare_parameter("drag_coef", 0.04)           # N / (m/s)^2  (quad ~15 cm, Cd·A pequeño)
        self.declare_parameter("gust_probability", 0.0015)  # per tick (~cada 7 s a 100 Hz)
        self.declare_parameter("gust_amp_min", 0.4)         # N  (~0.04 g lateral)
        self.declare_parameter("gust_amp_max", 1.5)         # N  (~0.14 g lateral)
        self.declare_parameter("gust_dur_min", 0.3)         # s
        self.declare_parameter("gust_dur_max", 1.2)         # s
        self.declare_parameter("force_limit", 3.0)          # N  (clamp global de seguridad)
        self.declare_parameter("seed", -1)

        gp = self.get_parameter
        quad_name = gp("quad_name").value
        rate = float(gp("publish_rate").value)
        level = gp("turbulence_level").value
        seed = int(gp("seed").value)
        if seed >= 0:
            random.seed(seed)
            np.random.seed(seed)

        if level not in TURBULENCE_LEVELS:
            self.get_logger().warn(f"unknown turbulence_level '{level}', using 'moderate'")
            level = "moderate"
        sigma_turb, L_turb = TURBULENCE_LEVELS[level]

        mean_speed = float(gp("mean_wind_speed").value)
        self.mean_dir = self._random_horizontal_dir()
        self.mean_speed_ou = OrnsteinUhlenbeck(
            tau=float(gp("mean_wind_tau").value),
            sigma=float(gp("mean_wind_sigma").value),
            x0=mean_speed,
        )
        self.mean_wind_base = mean_speed
        self.turb = DrydenTurbulence(sigma=sigma_turb, L=L_turb, V=max(mean_speed, 1.0))
        self.gust = Gust()

        self.drag = float(gp("drag_coef").value)
        self.gust_prob = float(gp("gust_probability").value)
        self.gust_amp_range = (float(gp("gust_amp_min").value), float(gp("gust_amp_max").value))
        self.gust_dur_range = (float(gp("gust_dur_min").value), float(gp("gust_dur_max").value))
        self.force_limit = float(gp("force_limit").value)

        self.dt = 1.0 / rate
        topic = f"/{quad_name}/external_force"
        self.pub = self.create_publisher(ExternalForce, topic, 10)
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"Wind publisher started: topic={topic}, level={level}, "
            f"mean_speed={mean_speed} m/s, rate={rate} Hz"
        )

    @staticmethod
    def _random_horizontal_dir() -> np.ndarray:
        a = random.uniform(0.0, 2.0 * math.pi)
        return np.array([math.cos(a), math.sin(a), 0.0])

    def _tick(self):
        # Slowly drift mean wind direction
        if random.random() < 0.001:  # ~every 10 s on average
            self.mean_dir = self._random_horizontal_dir()

        # Mean wind speed with OU perturbation around base
        speed = self.mean_wind_base + 0.3 * self.mean_speed_ou.step(self.dt)
        speed = max(speed, 0.0)
        v_wind = speed * self.mean_dir + self.turb.step(self.dt)

        # Quadratic-drag force from wind (assumes drone roughly stationary in world)
        v_mag = float(np.linalg.norm(v_wind))
        f_drag = self.drag * v_mag * v_wind  # N

        # Discrete gust
        if self.gust.remaining <= 0.0 and random.random() < self.gust_prob:
            amp = random.uniform(*self.gust_amp_range)
            dur = random.uniform(*self.gust_dur_range)
            self.gust.trigger(amp, dur)
            self.get_logger().info(f"gust triggered: amp={amp:.1f} N, dur={dur:.2f} s")
        f_gust = self.gust.step(self.dt)

        f_total = f_drag + f_gust

        # Safety clamp — never exceed force_limit (N)
        f_mag = float(np.linalg.norm(f_total))
        if f_mag > self.force_limit:
            f_total = f_total * (self.force_limit / f_mag)

        msg = ExternalForce()
        msg.duration = Duration(sec=0, nanosec=0)  # ignored in persistent mode
        msg.pos = Point(x=0.0, y=0.0, z=0.0)
        msg.force = Vector3(x=float(f_total[0]), y=float(f_total[1]), z=float(f_total[2]))
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WindPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish a zero force so the plugin clears residual wrench
        try:
            zero = ExternalForce()
            zero.duration = Duration(sec=0, nanosec=0)
            zero.pos = Point()
            zero.force = Vector3()
            node.pub.publish(zero)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
