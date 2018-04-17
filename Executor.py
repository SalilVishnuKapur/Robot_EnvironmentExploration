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
          # 1. TODO hardcode all the 6 Coordinate Points
          phases = {"Phase 1" : ()}
          # 2. TODO Initialize The Mapper and the Move class
          mapper = Mapping()
          mover = Move()

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
