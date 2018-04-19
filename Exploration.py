import math
from util import Util
from DangerChecker import DangerChecker
import collections

class Exploration:
      ################################################################################################
      # Functionalities of this module are as follows :-                                             #
      # 1. To find the Next position of the robot                                                    #                        #
      # 2. To Check if there is danger of rbot getting stuck in an alley or not being able to rotate #                         #
      ################################################################################################

      def __init__(self, st_x, st_y, ed_x, ed_y, dt, mapp, move):
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
          self.dangerChecker = DangerChecker()

      def distanceBetweenPoints(self, st_x, st_y, end_x, end_y):
          '''
          Finds the distance between two points.
          '''
          dis = ((end_y - st_y)**2 +(end_x - st_x)**2)**(1/2)
          return(dis)

      def slopeAngle(self, dy, dx):
          '''
          Returns the angle which is the slope of
          a line.
          '''
          angle = math.degrees(math.atan2(dy, dx))
          if(angle <= 0):
             angle += 360
          return(angle)

      def angleOrientation(self, angle):
        '''
        This is used to setup the orientation for every time the robot progresses.
        The orientation is 180 degree range towards to goal so that robot doesn't
        walk in the backward direction.
        '''
        if(90 <= angle <= 270):
           minTheta = int(angle - 90)
           maxTheta = int(angle + 90)
           limit = list(range(minTheta, maxTheta+1))
        elif(0 <= angle < 90):
           maxTheta = int(angle + 90)
           minTheta = 360 + int(angle - 90)
           limit = list(range(0, maxTheta+1)) + list(range(minTheta, 361))
        elif(270 < angle <= 360):
           maxTheta = 360 - int(angle + 90)
           minTheta = int(angle - 90)
           limit = list(range(minTheta, 360+1)) + list(range(0, maxTheta+1))
        return(limit)

      def infSetter(self):
          sensorOrientation = {}
          lt = []
          for key, val in self.inf.items():
              if(key in self.rangeAngles):
                 sensorOrientation[key] = val
          #sensorInf = dict(sorted(sensorOrientation.items()))
          sensorOrientation = collections.OrderedDict(sorted(sensorOrientation.items()))
          '''
          for key in sorted(sensorOrientation.keys()):
              lt.append((key, sensorOrientation[key]))
          for couple in lt:
              sensorInf[couple[0]] = couple[1]
          print(lt)
          '''
          '''
          print("***************************")
          print(sensorOrientation)
          print("***************************")
          '''
          return(sensorOrientation)


      def motionToGoal(self):
          '''
          Motion-to-goal: Move to current Oi to
          minimize G(x), until goal (success) or
          G(x) increases (local minima). Returns the
          point at which the G(x) is minimum.
          '''
          lt = []
          count = 0
          #print("----------------------")
          #print(self.inf)
          #print("----------------------")
          for key, val in self.inf.items():   
              if(val < 255.0):
                 if((key-10 in self.inf.keys()) and (key+10 in self.inf.keys())):
                    # 2 Distance Comparison
                    if(self.distanceBetweenPoints(self.start_x + self.inf[key-10] * math.cos(Util.deg2rad(key-10)), self.start_y + self.inf[key-10] * math.sin(Util.deg2rad(key-10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3 or  self.distanceBetweenPoints(self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key)), self.start_x + self.inf[key+10] * math.cos(Util.deg2rad(key+10)), self.start_y + self.inf[key+10] * math.sin(Util.deg2rad(key+10))) > 3):
                       lt.append((self.start_x + val* math.cos(Util.deg2rad(key)), self.start_y + val * math.sin(Util.deg2rad(key))))
                 elif(key-10 in self.inf.keys()):
                    # 1 Distance Comparison
                    if(self.distanceBetweenPoints(self.start_x + self.inf[key-10] * math.cos(Util.deg2rad(key-10)), self.start_y + self.inf[key-10] * math.sin(Util.deg2rad(key-10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3):
                       lt.append((self.start_x + val* math.cos(Util.deg2rad(key)), self.start_y + val * math.sin(Util.deg2rad(key))))
                 elif(key+10 in self.inf.keys()):
                    # 1 Distance Comparison
                    if(self.distanceBetweenPoints(self.start_x + self.inf[key+10] * math.cos(Util.deg2rad(key+10)), self.start_y + self.inf[key+10] * math.sin(Util.deg2rad(key+10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3):
                       lt.append((self.start_x + val* math.cos(Util.deg2rad(key)), self.start_y + val * math.sin(Util.deg2rad(key))))
          minDistance = 99999999999999
          minPoint = (0,0)
          for point in lt:
                  dis = self.distanceBetweenPoints(self.start_x, self.start_y, point[0], point[1]) + self.distanceBetweenPoints(point[0], point[1], self.goal_x, self.goal_y)
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
          check = self.dangerChecker.check_alley_width(self.inf)
          return(check)

      def refereshMapping(self):
          '''
          This will call the Mapping module to get the updated mapping as per the updated position of robot
          '''
          #TODO:-Call the mapping method of the Mapper class

          data = self.mapper.update(self.mover)
          return(data)

      def triggerMovement(self,tempx,tempy):
          '''
          This will call the mover module to move the robot to its position a little
          '''
          #TODO:-Call the movement method of the Move class
          print("Trigger Movement to [" + str(tempx) + ", " + str(tempy) + "]")
          self.mover.waypoint(tempx, tempy)

      def controller(self):
          self.inf = self.refereshMapping()
          if(True or self.danger() == False):
              angle = self.slopeAngle(self.goal_y - self.start_y, self.goal_x - self.start_x)
              self.rangeAngles = self.angleOrientation(angle)
              self.inf = self.infSetter()
              #print(self.inf)
              self.start_x,self.start_y = self.motionToGoal()
              print("Inside controller. Going to: ",self.start_x,self.start_y)
              self.triggerMovement(self.start_x,self.start_y)
              return(self.controller())
          else:
              return(True)
