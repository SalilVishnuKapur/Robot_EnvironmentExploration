from util import Util as util
from Move import Move
import math
import time
import ev3dev as ev3

from ev3dev.ev3 import *

class Mapping:

    def __init__(self):

        '''MAKE SURE THE SENSOR IS POINTED FORWARD AT START OF CODE (on-robot jig needs to be designed to do this)'''
        self.mS = MediumMotor('outD')  # mS = motor_sensor for the ultrasonic sensor to be rotated
        self.ultra1 = UltrasonicSensor()
        self.sensor_range = 100
        self.gear_ratio = 28  # Motor must turn 28 times for 1 turn of sensor

        self.mS.position = 0  # The sensor is zeroed with relation to the robot, which starts at pi/2 degrees global.

        '''Angle resolution for scanning, in degrees'''
        self.scan_res = 10
        self.N_theta = int(360/self.scan_res)
        self.max_range = 255.0
        '''Initial Conditions, render a space'''
        maxX, maxY = (110, 100)  # Space is 103.0, 87.5 cm, so the rendered area is a bot larger to forgive misalignment

        minX = -maxX
        minY = -maxY

        res = 1  # cm
        numX = int((maxX - minX) / res)
        numY = int((maxX - minX) / res)

        self.X = util.linspace(minX, maxX, res)
        self.Y = util.linspace(minY, maxY, res)
        self.XX, self.YY = util.meshgrid(self.X, self.Y)

        '''A matrix containing all the grid points in the rendered space, an occupancy grid, , for an obstacle probability to 
           be assigned'''
        self.ZZ = []
        for elem in self.XX:
                new_list = []
                for elem2 in elem:
                	new_list.append(0)
                self.ZZ.append(new_list)

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
        X_min = []
        for element in self.X:
            X_min.append(math.fabs(element - x))

        Y_min = []
        for element in self.Y:
            Y_min.append(math.fabs(element - y))

        idxX = util.min(X_min)
        idxY = util.min(Y_min)

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

        if self.ZZ[x][y] > self.obs_threshold:
            obstruction = True
        else:
            obstruction = False

        return obstruction, self.ZZ[x][y]

    def update_occupancy_grid(self, robot_x, robot_y, polar_length, polar_angle):
        """
        Update the ZZ occupancy grid such that it may be used for mapping and path planning.

        :param robot_x: float, the x position of the robot in inches
        :param robot_y: float, the y position of the robot in inches
        :param polar_length: array, the length of each ultrasonic pulse from the robot in the update method
        :param polar_angle: array, the global angle that each polar_length element was recorded at.
        :return:
        """
 
        for idx, ray in enumerate(polar_length):

            '''If the ray is at the maximum sensor length, set it to infinity length to avoid false returns'''
            if ray is self.sensor_range:
                ray = 1000

            x_ray = robot_x + ray * math.cos(polar_angle[idx])  # cartesian components of the ray
            y_ray = robot_y + ray * math.sin(polar_angle[idx])

            x_idx, y_idx = self.coord_to_index(x_ray, y_ray)
		
            self.ZZ[y_idx][x_idx] = 1  # TODO: This is a great place to implement a sensor model.

    def update(self,move):
        """
        Turn the ultrasonic sensor to zero degrees global, and then scan 360 degrees, send the results to get updated in
        the occupancy grid.

        :return:
        """
        print('Updating world map')

        '''Turn motor to zero degrees in global coordinate'''
        robot_x, robot_y, robot_phi = move.pose()

        # No intelligent script to optimize angle to avoid wrapping the cord of the sensor
        sensor_desired_position = math.degrees(-robot_phi)
        motor_desired_position = -sensor_desired_position*self.gear_ratio  # negative because the gears are backwards
        self.mS.run_to_abs_pos(position_sp=motor_desired_position, speed_sp=1500, stop_action='hold')

        polar_length = []
        polar_angle = []

        sensor_theta = util.frange(0, 360, self.scan_res)
        for idx, theta in enumerate(sensor_theta):

            self.mS.run_to_abs_pos(position_sp=-(theta*self.gear_ratio+motor_desired_position), stop_action='hold')
            self.mS.wait_while('running')
            length = self.ultra1.distance_centimeters
            if length > self.max_range:
                length = 2*self.max_range  # Make the measurement inf incase the sensor doesn't read in range
            polar_length.append(length)
            polar_angle.append(theta)

        '''Turn sensor back to zero so the cord doesn't wrap'''
        self.mS.run_to_abs_pos(position_sp=0, speed_sp=1500, stop_action='hold')

        '''Send measurements for processing'''
        self.update_occupancy_grid(robot_x, robot_y, polar_length, polar_angle)
        #print(polar_length)
        '''Return 360 dict to exploration'''
        return dict(zip(polar_angle, polar_length))

