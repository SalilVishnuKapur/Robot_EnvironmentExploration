class DangerChecker:
    
   def __init__(self):
       self.threshold = 100
       self.min_points = 10
   def check_alley_width(self, scan_points):
       '''
       This will check for the folowing two conditions
       1. Whether the alley allows to get robot in
       2. If it is possible to take a turn
       '''
       num_of_points = 0
       threshold = self.threshold
       max_num_of_points = 0
       for point in scan_points:
           if point>threshold:
               num_of_points += 1
           else:
               if num_of_points>max_num_of_points:
                   max_num_of_points = num_of_points
               num_of_points = 0
       if num_of_points>max_num_of_points:
           max_num_of_points = num_of_points
       return max_num_of_points>self.min_points
