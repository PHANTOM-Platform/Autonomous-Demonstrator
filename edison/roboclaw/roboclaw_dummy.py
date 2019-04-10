import logging
import time

logger = logging.getLogger('pyroboclaw')
logger.setLevel(logging.INFO)


class RoboClawDummy:
    def __init__(self, port, address, auto_recover=False, **kwargs):
        self.motor_speeds = [0, 0]
        self.encoders = [0, 0]
        logger.warning("ROBOCLAW DUMMY")

    def set_speed(self, motor, speed):
        self.motor_speeds[motor-1] = speed
        logger.info('set motor {} to speed {}'.format(motor, speed))

    def recover_serial(self):
        logger.info('recovering serial')

    def drive_to_position_raw(self, motor, accel, speed, deccel, position, buffer):
        logger.info('driving to raw position')

    def drive_to_position(self, motor, accel, speed, deccel, position, buffer):
        logger.info('driving to position')

    def drive_motor(self, motor, speed):
        # assert -64 <= speed <= 63
        write_speed = speed + 64
        self.motor_speeds[motor - 1] = speed*2
        logger.info('drive motor {} to speed {} (writing {})'.format(motor, speed*2, write_speed))

    def drive_motor_signed(self, motor, speed):
        if speed < 0:
            self.drive_motor_backward(motor, -speed)
        else:
            self.drive_motor_forward(motor, -speed)

    def drive_motor_forward(self, motor, speed):
        self.motor_speeds[motor - 1] = speed
        logger.info('drive motor {} to forward speed {}'.format(motor, speed))

    def drive_motor_backward(self, motor, speed):
        self.motor_speeds[motor - 1] = -speed
        logger.info('drive motor {} to backward speed {}'.format(motor, -speed))

    def stop_motor(self, motor):
        self.motor_speeds[motor - 1] = 0
        logger.info('stopping motor {}'.format(motor))

    def stop_all(self):
        self.motor_speeds[0] = 0
        self.motor_speeds[1] = 0
        logger.info('stopping all motors')

    def read_encoder(self, motor):
        # Currently, this function doesn't check over/underflow, which is fine since we're using pots.
        return self.encoders[motor-1]

    def reset_quad_encoders(self):
        self.encoders[0] = 0
        self.encoders[1] = 0

    def read_range(self, motor):
        logger.info('reading range of motor {}'.format(motor))
        return 0, 0

    def read_position(self, motor):
        # returns position as a percentage across the full set range of the motor
        logger.info('reading position of motor {}'.format(motor))
        encoder = self.read_encoder(motor)
        range = self.read_range(motor)
        return ((encoder - range[0]) / float(range[1] - range[0])) * 100.

    def read_status(self):
        logger.info('reading status')
        return 'Normal'

    def read_temp_sensor(self, sensor):
        logger.info('reading temperature sensor {}'.format(sensor))
        return 0

    def read_batt_voltage(self, battery):
        if battery in ['logic', 'Logic', 'L', 'l']:
            logger.info('reading logic battery')
        else:
            logger.info('reading main battery')
        return 0

    def read_voltages(self):
        logger.info('reading both batteries')
        return 0, 0

    def read_currents(self):
        logger.info('reading currents')
        return 0, 0

    def read_motor_current(self, motor):
        logger.info('reading current of motor {}'.format(motor))
        return 0

    def read_motor_pwms(self):
        logger.info('reading motor PWMs')
        return 0, 0

    def read_motor_pwm(self, motor):
        logger.info('reading PWM of motor {}'.format(motor))
        return 0

    def read_input_pin_modes(self):
        logger.info('reading pin modes')
        return 'Default', 'Disabled', 'Disabled'

    def read_max_speed(self, motor):
        logger.info('reading max speed of motor {}'.format(motor))
        return 255

    def read_speed(self, motor):
        # returns velocity as a percentage of max speed
        logger.info('reading speed of motor {}'.format(motor))
        return self.motor_speeds[motor-1] / 2.55

    def drive_speed(self, speed1, speed2):
        logger.info('driving to speeds {}'.format((speed1, speed2)))

    def drive_speed_accel(self, speed1, speed2, accel):
        logger.info('driving to speeds {} with acceleration {}'.format((speed1, speed2), accel))

    def drive_speed_distance(self, speed1, speed2, dist1, dist2, buffer):
        logger.info('driving to speeds {} for distance {}'.format((speed1, speed2), (dist1, dist2)))

    def drive_speed_accel_distance(self, speed1, speed2, accel, dist1, dist2, buffer):
        logger.info('driving to speeds {} with acceleration {} for distance {}'.format((speed1, speed2), accel, (dist1, dist2)))

    def read_buffer_length(self):
        logger.info('reading buffer lengths')
        return 0x80, 0x80
