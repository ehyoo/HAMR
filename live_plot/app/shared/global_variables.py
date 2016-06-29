# Global Variables used throughout the app. 
# TODO: Global variables are bad. Very bad. Change these eventually. 
pause_graph = False
xdata = None
ydata = None
lines = None 
continue_reading  # If this is true, then the thread_read thread will not exit
reading = False # while this is true, thread_read is still alive
device = None # Object returned from hamr_serial