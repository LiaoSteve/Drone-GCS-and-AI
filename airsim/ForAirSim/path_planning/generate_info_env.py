import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys, os

def save_cube_2D_location(env_info_name, num, x, y, z, dx, dy, dz):       
    xx = [x, x, x + dx, x + dx]
    yy = [y, y + dy, y + dy, y]  
    f= open(env_info_name,"a") 
    f.write('cube '+str(num+1) + '\n')  
    for i in range(xx.__len__()):                            
        f.write(str(xx[i])+' '+str(yy[i])+' \n')    
    f.close()                
    
def get_object_position_from_txt(path):
    try:
        wp = []
        f = open(path,'r')
        while 1:
            line = f.readline()
            if line is not '':            
                text = line.split(' ')
                wp.append( [ float(text[0]), float(text[1]), float(text[2]) ] )
                print(text)
            else:
                break
        f.close()
        return wp
    except Exception as e:
        print(e)
        sys.exit(1)  

def get_path_data_from_txt(path):    
    wp = []
    f = open(path,'r')
    while 1:
        line = f.readline()
        if line is not '':            
            text = line.split(' ')
            wp.append( [ float(text[0]), float(text[1]), float(text[2]) ] )
            print(text)
        else:
            break
    f.close()
    return wp
      
        
if __name__ == '__main__':
    my_env = ['A', 'B','F', 'G']
    for num_env in range(my_env.__len__()):
        env_info_name = 'env_' + my_env[num_env] +'_info.txt'
        ob = get_object_position_from_txt('../record_position/object_' + my_env[num_env] +'.txt')    
        wp = get_path_data_from_txt('../record_position/wp_' + my_env[num_env] +'.txt') 
        # Save start point and land point
        f= open(env_info_name,"a") 
        f.write('start point ' + '\n')     
        f.write(str(0) + ' ' + str(0) + '\n') 
        f.close()                 
        # Save cube 2D location
        for i in range(len(ob)): 
            if my_env[num_env] == 'A':
                l = 15 
            if my_env[num_env] == 'B':
                l = 10
            if my_env[num_env] == 'F':
                l = 6
            save_cube_2D_location(env_info_name, i, ob[i][0], ob[i][1], ob[i][2], l/2, l/2, l/2) 
        # Save wp 2D location
        for i in range(len(wp)):
            try:            
                f= open(env_info_name,"a")   
                f.write('waypoint '+str(i+1) + '\n')                       
                f.write(str(wp[i][0])+' '+str(wp[i][1])+' \n')    
                f.close()         
            except Exception as e:
                print('error: ',e)        
                pass        