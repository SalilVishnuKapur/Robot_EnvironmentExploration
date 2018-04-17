import matplotlib.pyplot as plt
import numpy as np
import ev3dev.ev3 as ev3


def saturate(x, xmin, xmax):
    """
    Saturate or 'check' a value so that it lies between a predetermined range such that: min <= x <= max

    :param x: float
    :param xmin: float
    :param xmax: float
    :return: x, float, bounded to a range
    """
    if x < xmin:
        x = xmin
    elif x > xmax:
        x = xmax

    return x


def wrap_angle(angle):
    """
    similar to saturate, make sure an angle is within 0 and 2pi radians such that math may be performed on it.

    :param angle: float, radians
    :return: angle, float, bounded between 0 and 2pi radians
    """
    while angle < 0:
        angle = angle + 2*np.pi
    while angle > 2*np.pi:
        angle = angle - 2*np.pi

    return angle


class Mapping:

    def __init__(self):

        '''MAKE SURE THE SENSOR IS POINTED FORWARD AT START OF CODE (on-robot jig needs to be designed to do this)'''
        self.mS = ev3.MediumMotor('outD')  # mS = motor_sensor for the ultrasonic sensor to be rotated
        self.ultra1 = ev3.UltrasonicSensor()
        self.sensor_range = 100

        self.mS.position = 0  # The sensor is zeroed with relation to the robot, which starts at pi/2 degrees global.

        '''Angle resolution for scanning, in degrees'''
        scan_res = 2.5
        self.N_theta = int(360/scan_res)

        '''Initial Conditions, render a space'''
        maxX, maxY = (30, 20)
        minX = -maxX
        minY = -maxY

        res = 1
        numX = int((maxX - minX) / res)
        numY = int((maxX - minX) / res)

        X = np.linspace(minX, maxX, num=numX + 1)
        Y = np.linspace(minY, maxY, num=numY + 1)
        self.XX, self.YY = np.meshgrid(X, Y)

        '''A matrix containing all the grid points in the rendered space, an occupancy grid, , for an obstacle probability to 
           be assigned'''
        self.ZZ = np.zeros_like(self.XX)

        '''Threshold for obstacle detection'''
        self.obs_threshold = 0.2

    def coord_to_index(self, x, y):
        """
        Take in the meshgrid XX and YY matrixes, and a point of interest. Return the closest coordinate on the meshgrid in
        terms of meshgrid indices.

        :param x: float
        :param y: float
        :return:
        """

        idxX = (np.abs(self.XX[1, :] - x)).argmin()
        idxY = (np.abs(self.YY[:, 1] - y)).argmin()

        return idxX, idxY

    def test_index(self, y, x):
        """
        test an occupancy grid mesh point to see if there is an obstacle there or not, based off of the threshold value set
        in the initial conditions.

        :param y: float, the x-axis coordinate on the meshgrid (meshgrids transpose direction with their indexes)
        :param x: float, the y-axis coordinate on the meshgrid (meshgrids transpose direction with their indexes)
        :return: obstruction: bool, if the x,y point is an obstacle above threshold
        :return: ZZ(x,y): float, the probability there is an obstacle at the given coordinate
        """

        if self.ZZ[x, y] > self.obs_threshold:
            obstruction = True
        else:
            obstruction = False

        return obstruction, self.ZZ[x, y]

    def update_occupancy_grid(self, robot_x, robot_y, polar_length, polar_angle):
        """
        Update the ZZ occupancy grid such that it may be used for mapping and path planning.

        :param robot_x: float, the x position of the robot in inches
        :param robot_y: float, the y position of the robot in inches
        :param polar_length: array, the length of each ultrasonic pulse from the robot in the update method
        :param polar_angle: array, the global angle that each polar_length element was recorded at.
        :return:
        """

        for theta, ray in polar_angle, polar_length:

            '''If the ray is at the maximum sensor length, set it to infinity length to avoid false returns'''
            if ray is self.sensor_range:
                ray = 1000

            x_ray = robot_x + ray * np.cos(theta)  # cartesian components of the ray
            y_ray = robot_y + ray * np.sin(theta)

            x_idx, y_idx = self.coord_to_index(x_ray, y_ray)

            self.ZZ[y_idx, x_idx] = 1  # TODO: This is a great place to implement a sensor model.


    def update(self):
        """
        Turn the ultrasonic sensor to zero degrees global, and then scan 360 degrees, send the results to get updated in
        the occupancy grid.

        :return:
        """

        '''Turn motor to zero degrees in global coordinate'''
        robot_x, robot_y, robot_phi = RoboMove.pose()

        # No intelligent script to optimize angle to avoid wrapping the cord of the sensor
        self.mS.run_to_abs_pos(position_sp=np.rad2deg(-robot_phi), stop_action='hold')

        polar_length = np.array([])
        polar_angle = np.array([])

        sensor_theta = np.linspace(0, 360, self.N_theta)

        for idx, theta in enumerate(sensor_theta):

            self.mS.run_to_rel_pos(position_sp=theta, stop_action='hold')
            polar_length = np.append(polar_length, self.ultra1.distance_inches)
            polar_angle = np.append(polar_angle, theta)

        '''Turn sensor back to zero so the cord doesn't wrap'''
        self.mS.run_to_abs_pos(position_sp=0, stop_action='hold')

        '''Send measurements for processing'''
        self.update_occupancy_grid(robot_x, robot_y, polar_length, polar_angle)


class Move:

    def __init__(self):

        x = 0
        y = 0
        phi = np.pi/2  # Given the robot begins facing north, it starts at 90 degrees.

        self.mR = ev3.LargeMotor('outA')
        self.mL = ev3.LargeMotor('outB')

        self.mR.ramp_down_sp = 1  # Take 1 full second to start/stop to avoid wheel slip from over-torque
        self.mL.ramp_down_sp = 1
        self.mR.ramp_up_sp = 1
        self.mL.ramp_up_sp = 1

        # in mm
        self.axle_length = 59.5  # TODO: Sort out units (inches, mm are used in prevalance)
        self.radius_wheel = 30.0

        self.x = np.array(x)
        self.y = np.array(y)
        self.phi = np.array(phi)

        self.rotation_tol = np.deg2rad(1)  # Tolerance as to when to stop turning
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

        rel_angle = np.arctan2(np.sin(phi-angle), np.cos(phi-angle))

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

        angle = wrap_angle(angle)
        x, y, phi = self.pose()

        rel_angle = self.get_rel_angle(phi, angle)

        '''
        Get desired encoder position to complete the turn from arc_len = r*theta[rad] and 
        counts = (arc_len/wheel_rad)*(180/pi)
        '''
        arc_length_to_turn = abs(self.axle_length/2*rel_angle)
        counts_in_turn = (arc_length_to_turn/self.radius_wheel)*(180/np.pi)

        if rel_angle > 1:  # turn clockwise

            self.mL.run_to_rel_pos(position_sp=counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')
            self.mR.run_to_rel_pos(position_sp=-counts_in_turn, speed_sp=-self.turn_speed, stop_action='hold')

        else:  # Turn CCW

            self.mL.run_to_rel_pos(position_sp=-counts_in_turn, speed_sp=-self.turn_speed, stop_action='hold')
            self.mR.run_to_rel_pos(position_sp=counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')

        self.phi = angle

    def waypoint_dumb(self, x_wp, y_wp):
        """
        Drive in a straight line to a specified waypoint, dumb because there's no object avoidance.

        :param x_wp: float, x coordinate to move to
        :param y_wp: float, y coordinate to move to
        :return: none
        """
        # TODO: Add object bump detection to interrupt m.run_to_rel_pos while it moves.
        x, y, phi = self.pose()
        angle = np.arctan2((y_wp-y), (x_wp-x))
        self.turn(angle)

        dist = np.sqrt((x_wp-x)**2 + (y_wp - y)**2)

        # Turn dist into encoder counts using: counts = (dist/wheel_rad)*(180/pi)
        counts_to_wp = (dist/self.radius_wheel) * (180/np.pi)

        self.mL.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, stop_action='hold')
        self.mR.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, stop_action='hold')

        self.x, self.y = x_wp, y_wp


if __name__ == '__main__':

    point0 = [[0, 0]]
    point1 = [[3, 0]]
    point2 = [[0, 3]]
    point3 = [[3, 3]]
    waypoints = [point1, point2, point3]

    RoboMove = Move()
    RoboMap = Mapping()

    for wp in waypoints:
        print(wp)
        RoboMove.waypoint_dumb(wp[0][0], wp[0][1])
        RoboMap.update()
