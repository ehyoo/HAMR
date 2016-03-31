import os
from time import gmtime, strftime


# Open new file in the given directory. Create new directory if it does not exist
# save data in CSV format (opens in Excel)
# PARAMETERS: name = string to prepend to output file
#             dir = string to name directory of output files
#             folder_date = True if today's date should be appended to directory
# RETURN: a file object - https://docs.python.org/2/library/stdtypes.html#bltin-file-objects
def open_file(name, dir, folder_date = False):
    script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
    filename = '_'.join((name, strftime("[%Y-%m-%d]_[%H-%M-%S]") + '.csv'))
    folder_path = os.path.join(script_dir, dir)
    if folder_date: folder_path +=  strftime("[%Y-%m-%d]")
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
    
    file_path = os.path.join(folder_path, filename)
    new_file = open(file_path, 'w')
    return new_file