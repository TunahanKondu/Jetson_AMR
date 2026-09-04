"""PID steering controller independent from ROS interfaces."""


class PIDController:
    """Calculate a bounded PID output with integral anti-windup."""

    def __init__(
        self,
        kp,
        ki,
        kd,
        output_limit,
        integral_limit,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.output_limit = abs(float(output_limit))
        self.integral_limit = abs(float(integral_limit))
        self.reset()

    def update(self, error, dt):
        """Return the PID output for an error and elapsed time."""
        error = float(error)
        dt = float(dt)
        if dt <= 0.0:
            raise ValueError('dt must be greater than zero')

        self.integral += error * dt
        self.integral = _clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )

        derivative = 0.0
        if self.previous_error is not None:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error

        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        return _clamp(output, -self.output_limit, self.output_limit)

    def reset(self):
        """Clear the accumulated and previous error state."""
        self.integral = 0.0
        self.previous_error = None


def _clamp(value, minimum, maximum):
    return max(minimum, min(float(value), maximum))


def calculate_linear_speed(
    error,
    maximum_speed,
    minimum_speed,
    slowdown_gain,
):
    """Reduce forward speed as the normalized line error grows."""
    maximum_speed = float(maximum_speed)
    minimum_speed = float(minimum_speed)
    slowdown_gain = float(slowdown_gain)

    if minimum_speed < 0.0 or maximum_speed < minimum_speed:
        raise ValueError(
            'speeds must satisfy 0 <= minimum_speed <= maximum_speed'
        )
    if not 0.0 <= slowdown_gain <= 1.0:
        raise ValueError('slowdown_gain must be between zero and one')

    normalized_error = _clamp(abs(float(error)), 0.0, 1.0)
    target_speed = maximum_speed * (
        1.0 - slowdown_gain * normalized_error
    )
    return _clamp(target_speed, minimum_speed, maximum_speed)


def slew_rate_limit(target, current, maximum_rate, dt):
    """Limit how quickly a command may change without overshooting it."""
    target = float(target)
    current = float(current)
    maximum_rate = float(maximum_rate)
    dt = float(dt)

    if maximum_rate <= 0.0:
        raise ValueError('maximum_rate must be greater than zero')
    if dt <= 0.0:
        raise ValueError('dt must be greater than zero')

    maximum_change = maximum_rate * dt
    change = _clamp(target - current, -maximum_change, maximum_change)
    return current + change
