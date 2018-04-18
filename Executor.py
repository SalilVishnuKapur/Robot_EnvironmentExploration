import math
from Exploration import Exploration
from Mapper import Mapping
from Move import Move
import ev3dev as ev3

class Executor:
      ################################################################################################
      # Functionalities of this module are as follows :-                                             #
      # 1. This is the executor script which initiates the Robot Exploration                         #
      ################################################################################################
      if __name__ == '__main__':
          '''Phases for the robot to attempt to travel around the space'''
          # Robot space is 2*103, 2*87.5cm
          centre_phase = [0,0]

          # The 7 Phases
          phases = {"Phase 1" : [(53, -112.5), (53, -87.5)], "Phase 2" : [(53, -87.5), (90, -80)],
              "Phase 3" : [(90, -80), (90, 80)], "Phase 4" : [(90, 80), (-90, 80)],
              "Phase 5" : [(-90, 80), (-90, -80)], "Phase 6" : [(-90, -80), (53, -87.5)],
              "Phase 7" : [(53, -87.5), (53, -112.5)]}

          #Init Objects
          mapper = Mapping()
          move = Move()

          # Iterating over the 7 phases
          for pointer, (key,phase) in enumerate(phases.items()):
              print(phase)
              values = phase
              From = values[0]
              To = values[1]
              explore = Exploration(From[0], From[1], To[0], To[1], {}, mapper, move)
              explore = explore.controller()
              if(explore == True):
                  print("Successfully Traversed Phase "+ str(pointer))
              else:
                  print("Terminating Exploration of Environment")
                  print("No. of phases explored :- "+ str(pointer))
                  break
