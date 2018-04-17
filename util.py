import math

class Util:
    def deg2rad(x):
        return(x * math.pi / 180)

    def custom_meshgrid(list1,list2):
       list1_length = len(list1)
       list2_length = len(list2)
       XX_c = []
       for i in range(list2_length):
           XX_c.append(list1)
       YY_c = []
       for x in list2:
           list1 = []
           for i in range(list1_length):
               list1.append(x)
           YY_c.append(list1)
       return XX_c,YY_c
