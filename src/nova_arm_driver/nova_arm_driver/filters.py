#
## filters.py
#
#  Reusable filter classes for target smoothing and signal filtering.
#

import math


class EMAFilter:
    """Exponential Moving Average (EMA) filter."""

    def __init__(self, alpha: float = 0.05, initial_value: float = 0.0):
        self.alpha = float(alpha)
        self.value = float(initial_value)

    def update(self, measurement: float) -> float:
        self.value += self.alpha * (float(measurement) - self.value)
        return self.value

    def reset(self, initial_value: float = 0.0):
        self.value = float(initial_value)


class KalmanFilter1D:
    """1D Discrete Kalman Filter for scalar tracking (angle positioning)."""

    def __init__(
        self,
        process_noise: float = 0.05,
        measurement_noise: float = 2.0,
        initial_value: float = 0.0,
        initial_uncertainty: float = 1.0,
    ):
        self.x = float(initial_value)  # Estimated state
        self.P = float(initial_uncertainty)  # Estimate error covariance
        self.Q = float(process_noise)  # Process noise covariance
        self.R = float(measurement_noise)  # Measurement noise covariance

    def predict(self):
        """Prediction stage assuming zero velocity / static state model."""
        self.P += self.Q

    def update(self, measurement: float) -> float:
        """Correction stage given a new raw target measurement."""
        self.predict()

        # Compute Kalman Gain
        K = self.P / (self.P + self.R)

        # Update State & Covariance
        self.x += K * (float(measurement) - self.x)
        self.P *= 1.0 - K

        return self.x

    def reset(
        self, initial_value: float = 0.0, initial_uncertainty: float = 1.0
    ):
        self.x = float(initial_value)
        self.P = float(initial_uncertainty)


class OneEuroFilter:
    """Adaptive low-pass filter (1€ Filter) balancing speed and jitter suppression."""

    def __init__(
        self,
        dt: float = 0.05,
        min_cutoff: float = 1.0,
        beta: float = 0.0,
        d_cutoff: float = 1.0,
        initial_value: float = 0.0,
    ):
        self.dt = float(dt)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)

        self.x_prev = float(initial_value)
        self.dx_prev = 0.0

    def smoothing_factor(self, cutoff: float) -> float:
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / self.dt)

    def lowpass(self, current: float, previous: float, alpha: float) -> float:
        return previous + alpha * (current - previous)

    def update(self, measurement: float) -> float:
        measurement = float(measurement)

        # Raw derivative (speed estimate)
        dx = (measurement - self.x_prev) / self.dt

        # Filter derivative
        alpha_d = self.smoothing_factor(self.d_cutoff)
        dx_hat = self.lowpass(dx, self.dx_prev, alpha_d)

        # Adaptive cutoff frequency based on speed
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        alpha = self.smoothing_factor(cutoff)

        # Filter signal
        x_hat = self.lowpass(measurement, self.x_prev, alpha)

        self.x_prev = x_hat
        self.dx_prev = dx_hat

        return x_hat

    def reset(self, initial_value: float = 0.0):
        self.x_prev = float(initial_value)
        self.dx_prev = 0.0


class PassThroughFilter:
    """Bypasses filtering, returning the raw measurement directly."""

    def __init__(self, initial_value: float = 0.0):
        self.value = float(initial_value)

    def update(self, measurement: float) -> float:
        self.value = float(measurement)
        return self.value

    def reset(self, initial_value: float = 0.0):
        self.value = float(initial_value)