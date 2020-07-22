import time
def sum_cal(x,y):
    sum_num = 0
    for i in range(x,y):
        sum_num += i    
    return sum_num
start_time = time.time()
print(sum_cal(1,100000000))
print('Time used: {} sec'.format(time.time()-start_time))
import time
from numba import jit

@jit
def sum_cal(x,y):
    
    sum_num = 0
    for i in range(x,y):
        sum_num += i
    return sum_num

start_time = time.time()
print(sum_cal(1,100000000))
print('Time used: {} sec'.format(time.time()-start_time))