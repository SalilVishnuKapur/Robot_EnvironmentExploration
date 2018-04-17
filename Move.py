import math
from util import Util as util
import ev3dev as ev3

class Move:

    def __init__(self):

        x = 0
        y = 0
        phi = math.pi/2  # Given the robot begins facing north, it starts at 90 degrees.

        self.mR = ev3.LargeMotor('outA')
        self.mL = ev3.LargeMotor('outB')

        self.mR.ramp_down_sp = 1  # Take 1 full second to start/stop to avoid wheel slip from over-torque
        self.mL.ramp_down_sp = 1
        self.mR.ramp_up_sp = 1
        self.mL.ramp_up_sp = 1

        # in mm
        self.axle_length = 59.5  # TODO: Sort out units (inches, mm are used in prevalance)
        self.radius_wheel = 30.0

        self.x = x
        self.y = y
        self.phi = phi

        self.rotation_tol = math.radians(1)  # Tolerance as to when to stop turning
        self.turn_speed = 100  # speed to turn at, in deg/s

        self.fwd_speed = 100  # speed to drive forward at in deg/s

    def get_rel_angle(self, phi, angle):
        """
        Receive two angles, find the difference between their unit vectors (sin and cos terms) and the perform atan2 for
        the angle bounded on -pi <= rel_angle <= pi.

        :param angle: float, radians
        :param phi: float, radians
        :return: rel_angle: signed float, radians centered about phi of robot. (positive is CW)
        """

        rel_angle = math.atan2(math.sin(phi-angle), math.cos(phi-angle))

        return rel_angle

    def pose(self):
        """
        :return: floats, the pose of the robot, x, y, and its angle of rotation in radians.
        """

        return self.x, self.y, self.phi

    def turn(self, angle):
        """
        Turn the robot to a specified heading using the ev3dev built in closed loop control functions. Turns the robot
        the least angle possible to avoid incurring excess error.

        :param angle:
        :return:
        """

        angle = util.wrap_angle(angle)
        x, y, phi = self.pose()

        rel_angle = self.get_rel_angle(phi, angle)

        '''
        Get desired encoder position to complete the turn from arc_len = r*theta[rad] and 
        counts = (arc_len/wheel_rad)*(180/pi)
        '''
        arc_length_to_turn = abs(self.axle_length/2*rel_angle)
        counts_in_turn = (arc_length_to_turn/self.radius_wheel)*(180/math.pi)

        if rel_angle > 1:  # turn clockwise

            self.mL.run_to_rel_pos(position_sp=counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')
            self.mR.run_to_rel_pos(position_sp=-counts_in_turn, speed_sp=-self.turn_speed, stop_action='hold')

        else:  # Turn CCW

            self.mL.run_to_rel_pos(position_sp=-counts_in_turn, speed_sp=-self.turn_speed, stop_action='hold')
            self.mR.run_to_rel_pos(position_sp=counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')

        self.phi = angle

    def waypoint(self, x_wp, y_wp):
        """
        Drive in a straight line to a specified waypoint, dumb because there's no object avoidance.

        :param x_wp: float, x coordinate to move to
        :param y_wp: float, y coordinate to move to
        :return: none
        """
        # TODO: Add object bump detection to interrupt m.run_to_rel_pos while it moves.
        x, y, phi = self.pose()
        angle = math.atan2((y_wp-y), (x_wp-x))
        self.turn(angle)

        dist = math.sqrt((x_wp-x)**2 + (y_wp - y)**2)

        # Turn dist into encoder counts using: counts = (dist/wheel_rad)*(180/pi)
        counts_to_wp = (dist/self.radius_wheel) * (180/math.pi)

        self.mL.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, stop_action='hold')
        self.mR.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, stop_action='hold')

        self.x, self.y = x_wp, y_wp
