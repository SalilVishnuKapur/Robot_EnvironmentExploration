import math
from Exploration import Exploration
from Mapper import Mapping
from Move import Move


class Executor:
      ################################################################################################
      # Functionalities of this module are as follows :-                                             #
      # 1. This is the executor script which initiates the Robot Exploration                         #
      ################################################################################################
      if __name__ == '__main__':
          '''Phases for the robot to attempt to travel around the space'''
          # Robot space is 2*103, 2*87.5cm
          start_phase = [53, -112.5]  # Start the robot 25cm away from centre of door
          door_phase = [53, -87.5]
          b_right_corner_phase = [90, -80]  # bottom_right_corner_phase
          u_right_corner_phase = [90, 80]  # upper_right_corner_phase
          u_left_corner_phase = [-90, 80]
          b_left_corner_phase = [-90, -80]
          centre_phase = [0,0]

          '''Init Objects'''
          mapping = Mapping
          move = Move

          print("Hello") 

          for phase in range(1, 8):
              explore = Exploration(  , mapper, mover)  # 3. TODO Pass all the variable
              if(explore == True):
                  print("Successfully Traversed Phase "+ str(phase))
              else:
                  print("Terminating Exploration of Environment")
                  print("No. of phases explored :- "+ str(phase-1))
                  break
          '''
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
          '''
