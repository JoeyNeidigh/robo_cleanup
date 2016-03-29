import numpy as np

# Object class to model the messes in robo_cleanup
class Mess(object):

    def __init__(self, loc=(0,0), cb=0, cost=0):
        self.location = loc
        self.claimed_by = cb
        self.cost = cost

    # If the distance between self.location and other.location is less than .1 return true,
    # else return false
    def is_duplicate(self, other):
        return (np.sqrt((self.location[0] - other.location[0])**2 + (self.location[1] - other.location[1])**2) < .1)

    # If self.cost is higher than other.cost return true, else return false
    def cost_higher_than(self, other):
        return self.cost > other.cost
