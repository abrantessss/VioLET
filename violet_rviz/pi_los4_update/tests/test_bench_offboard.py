"""Ground Offboard test: never run the LOS4 law at zero speed."""
import math
import sys
import time
from test_controller_link import FakePx4


def rates(fake, after=0):
    return [m for m in fake.controls(after) if m.get_type() == 'SET_ATTITUDE_TARGET']


def test(executable):
    fake = FakePx4(executable, ['--controller-enabled', 'true', '--bench-offboard', 'true'])
    try:
        fake.air = False
        fake.speed = 0
        fake.position = False  # no GPS/local position needed for zero-rate bench test
        fake.pwm = 1500
        fake.wait(lambda: len(rates(fake)) > 5)
        time.sleep(1.4)
        fake.pwm = 1900
        fake.mode = 6  # RC/PX4 mode selection, not a command from the script
        time.sleep(0.5)
        assert all(m.thrust==0 and m.body_roll_rate==0 and m.body_pitch_rate==0 and m.body_yaw_rate==0
                   for m in rates(fake)), 'Nonzero bench control command'
        assert not any(m.get_type()=='COMMAND_LONG' and m.param2==6 for m in fake.controls()), 'Offboard entry requested'
        fake.kill_pwm = 1900
        fake.wait(lambda: fake.mode==1)
        end = time.monotonic()
        time.sleep(0.3)
        assert not rates(fake, end+0.15), 'Kill failed in bench mode'
    finally:
        output = fake.close()
    assert 'BENCH_OFFBOARD_ACTIVE' in output
    assert 'gamma_dot=0.0000' in output

    # The normal flight controller must still refuse on-ground / zero-speed use.
    fake = FakePx4(executable, ['--controller-enabled', 'true'])
    try:
        fake.air = False
        fake.speed = 0
        fake.pwm = 1500
        time.sleep(1.5)
        assert not rates(fake), 'Normal controller lost its flight guards'
    finally:
        fake.close()
    print('Bench ground preparation, zero-command enforcement, kill, and normal guards passed')


if __name__ == '__main__':
    test(sys.argv[1])
