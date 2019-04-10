import time
import math

WHEEL_RADIUS = 0.0625
WHEEL_DIAMETER = WHEEL_RADIUS * 2
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * math.pi
WHEEL_PULSES = 1200
WHEEL_DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / WHEEL_PULSES
WHEEL_ROTATIONS_PER_METRE = 1 / WHEEL_CIRCUMFERENCE
WHEEL_PULSES_PER_METRE = WHEEL_ROTATIONS_PER_METRE * WHEEL_PULSES
AXLE = 0.25
TARGET_SPEED = 30


class MotorControl:
    def __init__(self, tty, dummy=False):
        self.is_dummy = dummy
        if self.is_dummy:
            from roboclaw import RoboClawDummy as RoboClaw
        else:
            from roboclaw import RoboClaw
        self.roboclaw = RoboClaw(tty, 0x80)
        self.speeds = [0, 0]
        self.encoders = [0, 0]

    @staticmethod
    def pulses_to_metres(pulses):
        return pulses / WHEEL_PULSES_PER_METRE

    @staticmethod
    def metres_to_pulses(metres):
        return int(metres * WHEEL_PULSES_PER_METRE)

    def read_batt_voltage(self):
        return self.roboclaw.read_batt_voltage('')

    def reset_encoders(self):
        self.roboclaw.reset_quad_encoders()
        self.encoders = [0, 0]

    def read_encoders(self):
        enc_l, enc_r = self.roboclaw.read_encoders()
        self.encoders[0] = enc_l
        self.encoders[1] = enc_r
        return enc_l, enc_r

    def read_encoders_signed(self):
        (enc_l, enc_r) = self.read_encoders()
        if enc_l > 2**31:
            enc_l = enc_l - 2**32
        if enc_r > 2**31:
            enc_r = enc_r - 2**32
        return enc_l, enc_r

    def get_total_distance(self):
        (enc_l, enc_r) = self.read_encoders_signed()
        return enc_l * WHEEL_DISTANCE_PER_PULSE, enc_r * WHEEL_DISTANCE_PER_PULSE

    def stop_all(self):
        self.roboclaw.stop_all()

    def stop_all_buffered(self):
        self.drive_both_speed_distance(0, 0, 0)

    def set_speeds(self, speed_l, speed_r):
        self.speeds[0] = int(speed_l)
        self.speeds[1] = int(speed_r)
        self.roboclaw.drive_motor_signed(1, self.speeds[0])
        self.roboclaw.drive_motor_signed(2, self.speeds[1])

    def set_speed(self, speed):
        self.set_speeds(speed, speed)

    def accelerate_to_speeds(self, speed_l, speed_r, acceleration=100):
        start_l = self.speeds[0]
        start_r = self.speeds[1]
        target_l = speed_l
        target_r = speed_r
        diff_l = target_l - start_l
        diff_r = target_r - start_r
        inc_l = diff_l / acceleration
        inc_r = diff_r / acceleration
        curr_l = start_l
        curr_r = start_r

        self.set_speeds(start_l, start_r)

        for x in range(0, acceleration, 1):
            curr_l = curr_l + inc_l
            curr_r = curr_r + inc_r
            print((curr_l, curr_r), end=' ')
            self.set_speeds(curr_l, curr_r)
            time.sleep(0.01)
        print()

        self.set_speeds(target_l, target_r)

    def accelerate_to_speed(self, speed, acceleration=100):
        self.accelerate_to_speeds(speed, speed, acceleration)

    def drive_to_distance(self, distance, target_speed):
        speed_l = target_speed
        speed_r = target_speed
        enc_l, enc_r = self.read_encoders()
        target_l = int(enc_l + (distance * WHEEL_PULSES_PER_METRE))
        target_r = int(enc_r + (distance * WHEEL_PULSES_PER_METRE))

        self.set_speeds(speed_l, speed_r)

        while (speed_l != 0) or (speed_r != 0):
            enc_l, enc_r = self.read_encoders()
            if enc_l >= target_l:
                speed_l = 0
            if enc_r >= target_r:
                speed_r = 0
            self.set_speeds(speed_l, speed_r)

    def uturn_left(self, turn_radius, target_speed):
        k = 1 - (AXLE / (turn_radius + (AXLE * 0.5)))
        dist_l = 0
        dist_r = 0
        dist_c = 0
        theta = 0

        enc_start_l, enc_start_r = self.read_encoders()

        # self.set_speeds(target_speed, k * target_speed)
        self.drive_speed(int(k * target_speed), target_speed)

        while theta < math.pi:
            enc_l, enc_r = self.read_encoders()
            dist_r = WHEEL_DISTANCE_PER_PULSE * (enc_r - enc_start_r)
            dist_l = k * dist_r
            dist_c = 0.5 * (dist_l + dist_r)
            theta = (dist_r - dist_l) / AXLE  # theta in radians

        self.stop_all()

        return dist_l, dist_c, dist_r

    def uturn_right(self, turn_radius, target_speed):
        k = 1 - (AXLE / (turn_radius + (AXLE * 0.5)))
        dist_l = 0
        dist_r = 0
        dist_c = 0
        theta = 0

        enc_start_l, enc_start_r = self.read_encoders()

        # self.set_speeds(target_speed, k * target_speed)
        self.drive_speed(target_speed, int(k * target_speed))

        while theta < math.pi:
            enc_l, enc_r = self.read_encoders()
            dist_l = WHEEL_DISTANCE_PER_PULSE * (enc_l - enc_start_l)
            dist_r = k * dist_l
            dist_c = 0.5 * (dist_l + dist_r)
            theta = (dist_l - dist_r) / AXLE  # theta in radians

        self.stop_all()

        return dist_l, dist_c, dist_r

    def read_ispeeds(self):
        return self.roboclaw.read_ispeeds()

    def drive_both_speed(self, speed, buffer=0):
        self.roboclaw.drive_speed(speed, speed)

    def drive_speed(self, speed1, speed2):
        self.roboclaw.drive_speed(speed1, speed2)

    def drive_both_speed_accel(self, speed, accel):
        self.roboclaw.drive_speed_accel_distance(speed, speed, accel)

    def drive_speed_accel(self, speed1, speed2, accel):
        self.roboclaw.drive_speed_accel_distance(speed1, speed2, accel)

    def drive_both_speed_distance(self, speed, dist, buffer=0):
        self.roboclaw.drive_speed_distance(speed, speed, dist, dist, buffer)

    def drive_speed_distance(self, speed1, speed2, dist1, dist2, buffer=0):
        self.roboclaw.drive_speed_distance(speed1, speed2, dist1, dist2, buffer)

    def drive_both_speed_accel_distance(self, speed, accel, dist, buffer=0):
        self.roboclaw.drive_speed_accel_distance(speed, speed, accel, dist, dist, buffer)

    def drive_speed_accel_distance(self, speed1, speed2, accel, dist1, dist2, buffer=0):
        self.roboclaw.drive_speed_accel_distance(speed1, speed2, accel, dist1, dist2, buffer)

    def read_buffer_lengths(self):
        return self.roboclaw.read_buffer_length()

    def buffers_empty(self):
        buffers = self.read_buffer_lengths()
        return buffers[0] == 0x80 and buffers[1] == 0x80

    def wait_for_empty_buffers(self):
        while not self.buffers_empty():
            pass
