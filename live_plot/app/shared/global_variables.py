# Global Variables used throughout the app. 
# TODO: Global variables are bad. Very bad. Change these eventually. 
pause_graph = False
xdata = None
ydata = None
lines = None 
update_arduino_immediately = False
continue_reading = False # If this is true, then the thread_read thread will not exit
reading = False # while this is true, thread_read is still alive
device = None # Object returned from hamr_serial
thread_read = None # see arduino_functions- 
pause_graph = False 