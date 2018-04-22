import math
import time
import numpy as np
from util import Util as util

from ev3dev.ev3 import *

class Move:

    def __init__(self, initx, inity):
        global np
        
        x = initx
        y = inity
        phi = math.pi/2  # Given the robot begins facing north, it starts at 90 degrees.

        self.mR = LargeMotor('outB')
        self.mL = LargeMotor('outA')

        self.mR.ramp_down_sp = 1  # Take 1 full second to start/stop to avoid wheel slip from over-torque
        self.mL.ramp_down_sp = 1
        self.mR.ramp_up_sp = 1
        self.mL.ramp_up_sp = 1

        self.gyro = GyroSensor()
        self.bump = TouchSensor()

        self.gyro.mode = 'GYRO-ANG'
        self.gyro_initial = self.gyro.value()

        '''Turning Kalman filter info'''
        self.mu_turn = math.pi/2
        #self.S_turn = np.array([[0, 0], [0, 0]])
        self.S_turn = 1

        # in mm
        self.axle_length = (90/94.34)*13.97  # cm, TODO: Sort out units (inches, mm are used in prevalance)
        ''' Wheel radius calculation:
        0.04728723cm forward/encoder count (1 deg rotation=pi/180rad)
        forward = r*theta
        forward/theta = r
        '''
        self.radius_wheel = 0.04728723/(math.pi/180)

        self.x = x
        self.y = y
        self.phi = phi

        self.rotation_tol = math.radians(1)  # Tolerance as to when to stop turning
        self.turn_speed = 150  # speed to turn at, in deg/s

        self.fwd_speed = 150  # speed to drive forward at in deg/s

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

    def object_hit_routine(self, mapper_obj):
        """
        The robot has hit an object that has tripped the bump sensor. The robot has already recorded the location that
        this has occurred at. This method will back the robot up and also assign an obstacle on the occupancy grid to
        represent the obstacle that was hit, but evidently not detected by the ultrasonic sensor.

        :return: None.
        """
        bump_sensor_offset = 5  # cm, the distance from the bump sensor to the centre of the robot
        bumper_width = 14  # cm
        backup_dist = -15  # cm
        x, y, phi = self.pose()

        '''Step 1: Update the occupancy grid to reflect the hit object in the form of a line'''

        # Step 1a: Determine the centre of the bumper on the occupancy grid
        print("x,y,phi",x,y,phi)
        x_c = x + bump_sensor_offset * math.cos(phi)
        y_c = y + bump_sensor_offset * math.sin(phi)

        # Step 1b: Determine the left and right sides of the bumper on the occupancy grid
        x_l = x_c + bumper_width / 2 * math.cos(phi + math.pi)
        x_r = x_c + bumper_width / 2 * math.cos(phi - math.pi)
        y_l = y_c + bumper_width / 2 * math.sin(phi + math.pi)
        y_r = y_c + bumper_width / 2 * math.sin(phi - math.pi)

        # Step 1c: create two linspace objects to represent the x and y values of the bumper line
        N = bumper_width/(2*mapper_obj.resolution)  # Coarse resolution OK because update_grid_bump will overlap points to add gaussian uncertainty

        resx = (x_l - x_r) / N
        resy = (y_l - y_r) / N

        bumper_line_x = np.linspace(x_l, x_r, N)
        bumper_line_y = np.linspace(y_l, y_r, N)

        mapper_obj.update_grid_bump(bumper_line_x, bumper_line_y)

        '''Step 2: Back the robot up'''
        counts_to_wp = int((backup_dist / self.radius_wheel) * (180 / math.pi))
        print('[Object hit. Reversing ' + str(backup_dist) + ' cm]')

        self.mL.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, stop_action='hold')
        self.mR.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, stop_action='hold')



    def kalman_f_turn(self, rel_angle):
        """
        Perform Kalman filtering on a turn to update the robot phi angle.

        Method:
        Using the gyro as the sensor feedback. Predicted position is what was commanded by the turn method.

        :param angle:
        :return:
        """
        A = 1
        B = -1  # negative because +rel_angle is CW, whereas + phi is CCW
        C = 1  # again negative for the same reasons
        Q = math.radians(1.3)  # Motion noise
        R = math.radians(3)  # Measurement noise        

        '''Get Gyro reading'''
        gyro_present = -(self.gyro.value() - self.gyro_initial) + 90  # because the gyro readings are +90 degrees out of phase with the global axis
        gyro_abs = (util.wrap_angle(math.radians(gyro_present)))
        print('Gyro_abs: ',gyro_abs)

        '''Prediction'''
        mup = A*self.phi + B*rel_angle
        print('mup: ',mup)
        Sp = A*self.S_turn*A + Q
        print('Sp: ', Sp)        

        '''Filter implementation'''
        K = Sp*C/(C*Sp*C +  R)
        print('Kalman gain',K, ' C*mup', C*mup)
        print('K*(gyro_abs - C*mup): ', K*(gyro_abs-C*mup))
        self.mu_turn = mup + K * util.wrap_angle(gyro_abs - C*mup)  # Gyro returns angle in degrees
        self.S_turn = (1 - K*C)*Sp
        print("self.phi before",self.phi)
        print('self.mu_turn (variable out of k-filter)', self.mu_turn)
        self.phi = util.wrap_angle(self.mu_turn)
        print("rel_angle",rel_angle)
        print("self.phi after",self.phi)
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
        print('Turning robot ' + str(math.degrees(rel_angle)) + 'degrees')
        '''
        Get desired encoder position to complete the turn from arc_len = r*theta[rad] and 
        counts = (arc_len/wheel_rad)*(180/pi)
        '''
        arc_length_to_turn = abs((self.axle_length/2)*rel_angle)
        counts_in_turn = int((arc_length_to_turn/self.radius_wheel)*(180/math.pi))
        
        if(rel_angle > 1):  # turn clockwise
            self.mL.run_to_rel_pos(position_sp=counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')
            self.mR.run_to_rel_pos(position_sp=-counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')
            self.mL.wait_while('running')
            self.mR.wait_while('running')
        elif(counts_in_turn is 0):
            print("Don't do anything")
        else:  # Turn CCW
            self.mL.run_to_rel_pos(position_sp=-counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')
            self.mR.run_to_rel_pos(position_sp=counts_in_turn, speed_sp=self.turn_speed, stop_action='hold')
            self.mL.wait_while('running')
            self.mR.wait_while('running')
 
        return rel_angle

    def waypoint(self, x_wp, y_wp, mapper_obj):
        """
        Drive in a straight line to a specified waypoint, dumb because there's no object avoidance.

        :param x_wp: float, x coordinate to move to
        :param y_wp: float, y coordinate to move to
        :return: none
        """
        # TODO: Add object bump detection to interrupt m.run_to_rel_pos while it moves.
        x, y, phi = self.pose()
        angle = math.atan2((y_wp-y), (x_wp-x))

        dist = math.sqrt((x_wp-x)**2 + (y_wp - y)**2)
        print('### Move to Waypoint ###')
        print('Robot currently at [x, y, phi]: [' + str(x) + ', ' + str(y) + ', ' + str(math.degrees(phi)) + ']')
        print('Turning to ' + str(math.degrees(angle)) + ' and driving distance of ' + str(dist) + ' towards target')
        rel_angle = self.turn(angle)
        # Turn dist into encoder counts using: counts = (dist/wheel_rad)*(180/pi)
        counts_to_wp = int((dist/self.radius_wheel) * (180/math.pi))
        print('Drive forward ' + str(counts_to_wp) + ' encoder counts')

        start_counts_l = self.mL.position
        start_counts_r = self.mR.position
        
        st = time.time() 
        self.mL.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, ramp_down_sp=1, stop_action='hold')
        middle = time.time()
        self.mR.run_to_rel_pos(position_sp=counts_to_wp, speed_sp=self.fwd_speed, ramp_down_sp=1, stop_action='hold')
        end = time.time()

        print('Motor time delay Left: ', middle-st, ' Motor time delay Right: ', end-st)

        while self.mL.is_running:
            if self.bump.is_pressed:
                self.mL.stop(stop_action='hold')
                self.mR.stop(stop_action='hold')

                # Beep beep
                Sound.tone(620, 100)  # Tone, in Hz, duration in ms
                time.sleep(0.1)
                Sound.tone(620, 100)

                # Find position of left motor
                mL_x = x + ((self.mL.position - start_counts_l) / counts_to_wp) * dist * math.cos(angle)
                mL_y = y + ((self.mL.position - start_counts_l) / counts_to_wp) * dist * math.sin(angle)

                # Find position of right motor
                mR_x = x + ((self.mR.position - start_counts_r) / counts_to_wp) * dist * math.cos(angle)
                mR_y = y + ((self.mR.position - start_counts_r) / counts_to_wp) * dist * math.sin(angle)

                # Assign new robot current position to class variables
                self.x = (mL_x + mR_x) / 2
                self.y = (mL_y + mR_y) / 2

                self.object_hit_routine(mapper_obj)
                break
            else:
                self.x, self.y = x_wp, y_wp


        self.mR.wait_while('running')  # Wait for the right motor to stop running (shouldn't be much longer than left)

        time.sleep(0.5)
        self.kalman_f_turn(rel_angle)

        # TODO: Return the x and y values of where the robot is at for the exploration to make use of
        print('### Move Complete ###')
