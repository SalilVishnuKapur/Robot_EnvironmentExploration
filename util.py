import math

class Util:

    def deg2rad(x):
        return(x * math.pi / 180)

    def linspace_n(start, end, N):
        """
        REPLACE ALL FRANGE WITH linspace_n in Mapper.check_path()

        :param end:
        :param N:
        :return:
        """

        N = math.ceil(N)
        if start==end:
            L = []
            for idx in range(0, N):
                L.append(start)
            return L

        res = (end - start) / N

        #if res == 0:  # Condition for start == end
        #    ##should not arrive
        #    L = []
        #    for idx in range(0, N):
        #        L.append(start)
        L = []
        inc = res
        while 1:
           next = start + len(L) * inc
           if inc > 0 and next >= end:
              break
           elif inc < 0 and next <= end:
              break
           L.append(next)

        return L

    def linspace(min, max, res):
        """

        :param min: the minimum value of the list
        :param max:
        :param num:
        :return:
        """
        # res = int((max - min) / num)

        X = []
        print(min,max,res)
        for idx in range(min, max, res):
            X.append(idx)

        return X

    def meshgrid(list1, list2):
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


    def saturate(x, xmin, xmax):
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

    def wrap_angle(angle):
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

    def min(array):

        minimum_elem = 8000
        for idx, num in enumerate(array):
            if num < minimum_elem:
                minimum_elem = num
                minimum_idx = idx

        return minimum_idx

    def frange(start, end=None, inc=None):

    	if end == None:
        	end = start + 0.0
        	start = 0.0

    	if inc == None:
        	inc = 1.0
	
    	L = []
    	while 1:
        	next = start + len(L) * inc
        	if inc > 0 and next >= end:
            		break
        	elif inc < 0 and next <= end:
            		break
        	L.append(next)
        
    	return L
