import time
class TimeMe(object):
    def __init__(self, millis = 0):
        """Start a timer for the given amount of time"""
        self.millis = millis / 1000.0
        self.start_time = time.clock()

    def times_up(self):
        return self.millis < (time.clock() - self.start_time)

# a = TimeMe(millis= 1000)
# while not a.times_up(): pass
