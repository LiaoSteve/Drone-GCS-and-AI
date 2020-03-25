#------------  Use the .png of the folder to create the .gif
import os
import imageio

dirName1 = 'Data_A/'

dirs = os.listdir(dirName1) #--- change dirName 1 ,2
dirs.sort(key=lambda x:int(x[:-4]))
frames = []

gif_name1 = 'Data_gif_A.gif'

for file in dirs:    
    frames.append(imageio.imread(dirName1 + file)) #-- change dirName 1,2
    imageio.mimsave(gif_name1, frames, 'GIF', duration = 0.07) #-- change gif_name 1 ,2
    print (file) 
print('OK')