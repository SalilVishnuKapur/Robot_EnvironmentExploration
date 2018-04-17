import math
from util import Util

class Exploration:
      ################################################################################################
      # Functionalities of this module are as follows :-                                             #
      # 1. To find the Next position of the robot                                                    #                        #
      # 2. To Check if there is danger of rbot getting stuck in an alley or not being able to rotate #                         #
      ################################################################################################

      def init(self, st_x, st_y, ed_x, ed_y, dt, mapp, move):
          '''
          Variables initialization
          Parameters :-
          st_x : Starting position of robot's x coordinate
          st_y : Starting position of robot's y coordinate
          ed_x : Final Goal of robot's x coordinate
          ed_y : Final Goal of robot's y coordinate
          '''
          self.start_x = st_x
          self.start_y = st_y
          self.goal_x = ed_x
          self.goal_y = ed_y
          self.inf = dt
          self.mapper = mapp
          self.mover = move
          self.danger = DangerChecker()

      def distanceBetweenPoints(self):
          '''
          Finds the distance between two points.
          '''
          dis = ((self.goal_y - self.start_y)**2 +(self.goal_x - self.start_x)**2)**(1/2)
          return(dis)

      def motionToGoal(self):
          '''
          Motion-to-goal: Move to current Oi to
          minimize G(x), until goal (success) or
          G(x) increases (local minima). Returns the
          point at which the G(x) is minimum.
          '''
          lt = []
          count = 0
          for key, val in self.inf.items():
              if(val < 999):
                 if((key-10 in self.inf.keys()) and (key+10 in self.inf.keys())):
                    # 2 Distance Comparison
                    if(distanceBetweenPoints(self.start_x + self.inf[key-10] * math.cos(Util.deg2rad(key-10)), self.start_y + self.inf[key-10] * math.sin(Util.deg2rad(key-10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3 or  distanceBetweenPoints(self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key)), self.start_x + self.inf[key+10] * math.cos(Util.deg2rad(key+10)), self.start_y + self.inf[key+10] * math.sin(Util.deg2rad(key+10))) > 3):
                       lt.append((self.start_x + val* math.cos(Util.deg2rad(key)), self.start_y + val * math.sin(Util.deg2rad(key))))
                 elif(key-10 in self.inf.keys()):
                    # 1 Distance Comparison
                    if(distanceBetweenPoints(self.start_x + self.inf[key-10] * math.cos(Util.deg2rad(key-10)), self.start_y + self.inf[key-10] * math.sin(Util.deg2rad(key-10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3):
                       lt.append((self.start_x + val* math.cos(Util.deg2rad(key)), self.start_y + val * math.sin(Util.deg2rad(key))))
                 elif(key+10 in self.inf.keys()):
                    # 1 Distance Comparison
                    if(distanceBetweenPoints(self.start_x + self.inf[key-10] * math.cos(Util.deg2rad(key-10)), self.start_y + self.inf[key-10] * math.sin(Util.deg2rad(key-10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3):
                       lt.append((self.start_x + val* math.cos(Util.deg2rad(key)), self.start_y + val * math.sin(Util.deg2rad(key))))
          minDistance = 99999999999999
          minPoint = (0,0)
          for point in lt:
                  dis = distanceBetweenPoints(self.start_x, self.start_y, point[0], point[1]) + distanceBetweenPoints(point[0], point[1], self.goal_x, self.goal_y)
                  if(minDistance > dis):
                     minDistance =  dis
                     minPoint = point
          return(minPoint)

      def danger(self):
          '''
          This will check for the folowing two conditions
          1. Whether the alley allows to get robot in
          2. If it is possible to take a turn
          '''
          check = self.danger.check_alley_width(self.inf)
          return(check)

      def refereshMapping(self):
         '''
         This will call the Mapping module to get the updated mapping as per the updated position of robot
         '''
         #TODO:-Call the mapping method of the Mapper class
         data = self.mapper.update()
         return(data)

      def triggerMovement(self):
         '''
         This will call the mover module to move the robot to its position a little
         '''
         #TODO:-Call the movement method of the Move class
         self.mover.waypoint(self.goal_x, self.goal_y)

      def controller(self):
          self.inf = self.refereshMapping()
          if(self.danger(self.inf) == False):
              motionToGoal(self.self.start_x, self.self.start_y, self.end_x, self.end_y)
              self.triggerMovement()
              return(self.controller())
          else:
              return(True)
