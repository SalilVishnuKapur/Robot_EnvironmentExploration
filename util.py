import math

class Util:

    def deg2rad(x):
        return(x * math.pi / 180)

    def linspace(self, min, max, num):
        """


        :param min: the minimum value of the list
        :param max:
        :param num:
        :return:
        """
        res = (max - min) / num

        X = []
        for idx in range(min, max, res):
            X.append(idx)

        return X

    def meshgrid(self, list1, list2):
        """
        Usage:
        X = range(0, 10, 1)
        Y = range(1, 12, 1)
        XX, YY = custom_meshgrid(X, Y)
        """

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

        return XX_c, YY_c


    def saturate(self, x, xmin, xmax):
        """
        Saturate or 'check' a value so that it lies between a predetermined range such that: min <= x <= max

        :param x: float
        :param xmin: float
        :param xmax: float
        :return: x, float, bounded to a range
        """
        if x < xmin:
            x = xmin
        elif x > xmax:
            x = xmax

        return x


    def wrap_angle(self, angle):
        """
        similar to saturate, make sure an angle is within 0 and 2pi radians such that math may be performed on it.

        :param angle: float, radians
        :return: angle, float, bounded between 0 and 2pi radians
        """
        while angle < 0:
            angle = angle + 2*math.pi
        while angle > 2*math.pi:
            angle = angle - 2*math.pi

        return angle

    def min(self, array):

        minimum_elem = array[0]
        for num in array:
            if num < minimum_elem:
                minimum_elem = num

        return minimum_elem