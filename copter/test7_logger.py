import logging
from timeit import default_timer as timer

#logging.disable
error = logging.getLogger('CAM')

ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
ch.setFormatter(formatter)
error.addHandler(ch)

t = timer()
try:
    a=1/0
except Exception as e:
    error.critical(e)
    print(timer()-t)