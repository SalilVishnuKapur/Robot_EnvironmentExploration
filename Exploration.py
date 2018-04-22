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

      def __init__(self, st_x, st_y, ed_x, ed_y, dt, mapp, move, prevInf):
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
          self.priorInf = prevInf
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
             maxTheta = 90 - int(360 - angle)
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
          return(sensorOrientation)



      def rangeAnalytics(self):

          '''
          remainder = self.angle%10.0
          if(remainder == 0):
               if(self.angle == 0):
                  avg = (self.inf[10] + self.inf[0] + self.inf[350])/3
                  if(self.present_Distance_From_Goal < avg):
                     return(True)
               elif(self.angle == 360):
                  avg = (self.inf[10] + self.inf[0] + self.inf[350])/3
                  if(self.present_Distance_From_Goal < avg):
                     return(True)
               else:
                  avg = (self.inf[self.angle - 10] + self.inf[self.angle] + self.inf[self.angle + 10])/3
                  if(self.present_Distance_From_Goal < avg):
                     return(True)
          else:
               if(0 < self.angle < 10):
                  avg = (self.inf[self.angle - remainder] + self.inf[self.angle -remainder  + 10] )/2
                  if(self.present_Distance_From_Goal < avg):
                     return(True)
               elif(350 < self.angle < 360):
                  avg = (self.inf[self.angle - remainder] + self.inf[0])/2
                  if(self.present_Distance_From_Goal < avg):
                     return(True)
               else:
                  avg = (self.inf[self.angle - remainder] + self.inf[self.angle - remainder + 10])/2
                  if(self.present_Distance_From_Goal < avg):
                     return(True)
          return(False)
          '''
          #return
          return (self.mapper.check_path(self.start_x, self.start_y, self.goal_x, self.goal_y))

                  
      def motionToGoal(self):
          '''
          Motion-to-goal: Move to current Oi to
          minimize G(x), until goal (success) or
          G(x) increases (local minima). Returns the
          point at which the G(x) is minimum.
          '''
          lt = []
          count = 0
          print("Finding the next point to move")
          print(self.inf)
          print("Start x"+ str(self.start_x))
          print("Start y"+ str(self.start_y))
          print("Goal x"+ str(self.goal_x))
          print("Goal y"+ str(self.goal_y))
 
          # Here we are checking if there is no obstacle between initial to goal position
          if(self.rangeAnalytics()):
              return((self.goal_x, self.goal_y))

          # We find discontinuity if difference is greater than 3 and a deltaX of 5 is taken while calculating the pint to motion  
          for key, val in self.inf.items():   
              if(val < 255.0):
                 if((key-10 in self.inf.keys()) and (key+10 in self.inf.keys())):
                    # 2 Distance Comparison
                    if(self.distanceBetweenPoints(self.start_x + self.inf[key-10] * math.cos(Util.deg2rad(key-10)), self.start_y + self.inf[key-10] * math.sin(Util.deg2rad(key-10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3 or  self.distanceBetweenPoints(self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key)), self.start_x + self.inf[key+10] * math.cos(Util.deg2rad(key+10)), self.start_y + self.inf[key+10] * math.sin(Util.deg2rad(key+10))) > 3):
                       lt.append((self.start_x + (val-5)* math.cos(Util.deg2rad(key)), self.start_y + (val-5)* math.sin(Util.deg2rad(key))))
                 elif(key-10 in self.inf.keys()):
                    # 1 Distance Comparison
                    if(self.distanceBetweenPoints(self.start_x + self.inf[key-10] * math.cos(Util.deg2rad(key-10)), self.start_y + self.inf[key-10] * math.sin(Util.deg2rad(key-10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3):
                       lt.append((self.start_x + (val-5)* math.cos(Util.deg2rad(key)), self.start_y + (val-5) * math.sin(Util.deg2rad(key))))
                 elif(key+10 in self.inf.keys()):
                    # 1 Distance Comparison
                    if(self.distanceBetweenPoints(self.start_x + self.inf[key+10] * math.cos(Util.deg2rad(key+10)), self.start_y + self.inf[key+10] * math.sin(Util.deg2rad(key+10)), self.start_x + self.inf[key] * math.cos(Util.deg2rad(key)), self.start_y + self.inf[key] * math.sin(Util.deg2rad(key))) > 3):
                       lt.append((self.start_x + (val-5)* math.cos(Util.deg2rad(key)), self.start_y + (val-5) * math.sin(Util.deg2rad(key))))
          minDistance = 99999999999999
          minPoint = (self.goal_x, self.goal_y)
          for point in lt:
                  dis = self.distanceBetweenPoints(self.start_x, self.start_y, point[0], point[1]) + self.distanceBetweenPoints(point[0], point[1], self.goal_x, self.goal_y)
                  if(minDistance > dis):
                     minDistance =  dis
                     minPoint = point
          print("Found the next point to move which is :- ", minPoint)
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
          self.mover.waypoint(tempx, tempy, self.mapper)
 
      def checkPriorInf(self):
          if(self.priorInf == {}):
             return("Do_Mapper_Scan")
          else:
             self.inf = self.priorInf
             self.priorInf = {}

      def controller(self):
          if(self.checkPriorInf() == "Do_Mapper_Scan"):
              self.inf = self.refereshMapping()
          if(self.distanceBetweenPoints(self.start_x, self.start_y, self.goal_x, self.goal_y) > 10  and self.danger() == False):
              self.angle = self.slopeAngle(self.goal_y - self.start_y, self.goal_x - self.start_x)
              self.present_Distance_From_Goal = self.distanceBetweenPoints(self.start_x, self.start_y, self.goal_x, self.goal_y)
              self.rangeAngles = self.angleOrientation(self.angle)
              self.inf = self.infSetter()
              print("Robot's vision", self.inf)
              self.start_x,self.start_y = self.motionToGoal()
              print("Inside controller. Going to: ",self.start_x,self.start_y)
              self.triggerMovement(self.start_x,self.start_y)
              return(self.controller())
          else:
              return(self.inf, True)
