"""Display conversion only; values are not sent to the vehicle."""
import math

ACTUATOR_NAMES = ('motor', 'elevator', 'rudder', 'right_aileron', 'left_aileron')
DEFAULT_CHANNELS = (8, 4, 3, 2, 1)  # one-based MAIN output numbers


def normalized_output(value, minimum=1000.0, trim=1500.0, maximum=2000.0, encoding='pwm'):
    if not math.isfinite(value):
        return None
    if encoding == 'normalized':
        return max(-1.0, min(1.0, value))
    if encoding != 'pwm' or not minimum < trim < maximum:
        raise ValueError('Actuator encoding/calibration is invalid')
    if value <= 0:  # missing PWM output, not a valid servo deflection
        return None
    scale = maximum-trim if value >= trim else trim-minimum
    return max(-1.0, min(1.0, (value-trim)/scale))
