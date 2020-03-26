import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys, os

def plot_opaque_cube(ax1, x, y, z, dx, dy, dz): 
    xx = np.linspace(x, x+dx, 2)
    yy = np.linspace(y, y+dy, 2)
    zz = np.linspace(z, z+dz, 2)
    xx2, yy2 = np.meshgrid(xx, yy)
    ax1.plot_surface(xx2, yy2, np.full_like(xx2, z))
    ax1.plot_surface(xx2, yy2, np.full_like(xx2, z+dz))
   

    yy2, zz2 = np.meshgrid(yy, zz)
    ax1.plot_surface(np.full_like(yy2, x), yy2, zz2)
    ax1.plot_surface(np.full_like(yy2, x+dx), yy2, zz2)

    xx2, zz2= np.meshgrid(xx, zz)
    ax1.plot_surface(xx2, np.full_like(yy2, y), zz2)
    ax1.plot_surface(xx2, np.full_like(yy2, y+dy), zz2)
 

def plot_linear_cube(ax1, x, y, z, dx, dy, dz, color='red'):    
    #ax1 = Axes3D(fig)
    xx = [x, x, x+dx, x+dx, x]
    yy = [y, y+dy, y+dy, y, y]
    kwargs = {'alpha': 1, 'color': color}
    ax1.plot3D(xx, yy, [z]*5, **kwargs)
    ax1.plot3D(xx, yy, [z+dz]*5, **kwargs)
    ax1.plot3D([x, x], [y, y], [z, z+dz], **kwargs)
    ax1.plot3D([x, x], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax1.plot3D([x+dx, x+dx], [y+dy, y+dy], [z, z+dz], **kwargs)
    ax1.plot3D([x+dx, x+dx], [y, y], [z, z+dz], **kwargs)
    
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
        
if __name__ == '__main__':
    data_dir = 'Data_A'
    os.makedirs(data_dir, exist_ok=True)
    ob = get_object_position_from_txt('record_position/object_A.txt')    
    wp = np.array(get_path_data_from_txt('record_position/A.txt'))

    fig = plt.figure(figsize=(8,6))
    ax1 = fig.add_subplot(111, projection='3d')
    plt.ion()
    plt.show()
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
    ax1.set_zlabel('z')
    #plot_linear_cube(ax1,ob[0][0], ob[0][1], ob[0][2], 7.5, 7.5, 7.5)
    plot_opaque_cube(ax1,ob[0][0], ob[0][1], ob[0][2], 7.5, 7.5, 7.5)    
    ax1.scatter(wp[0,0], wp[0,1], -0.5, c='g', marker="$Start$", s= 1000) # marker>> https://matplotlib.org/3.1.1/api/markers_api.html#module-matplotlib.markers
    ax1.scatter(wp[-1,0], wp[-1,1], -0.5, c='r', marker="x", s= 80)
    ax1.set_xlim(-10, 80)
    ax1.set_ylim( 50,-40)
    ax1.set_zlim(0, 90) 
    ax1.view_init(azim=159, elev=23 )
    ax1.dist = 9 # zoom in/out
    plt.title('The result of fuzzy system avoidance', fontsize = 14)
    for i in range(len(wp)):
        ax1.scatter(wp[i,0], wp[i,1], -1*wp[i,2],c='b', marker="2", s=2)                                           
        plt.pause(0.0001)  
        plt.savefig(data_dir+'/'+str(i)+'.png')
    plt.savefig(data_dir+'_final.png')
    for i in range(50):
        ax1.view_init(azim=159+i, elev=23 )
        plt.pause(0.0001)
        plt.savefig(data_dir+'/'+str(len(wp))+str(i)+'.png')
    print('OK')
    plt.ioff()
    plt.show()