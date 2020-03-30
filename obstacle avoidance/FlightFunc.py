# -*- coding: utf-8 -*-
# Updating date : 2019/09/12
import time, sys, argparse, math
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil
import scipy
from scipy import special
import os

def get_distance_meters(location1,location2):
	dLat = location2.lat - location1.lat
	dLon = location2.lon - location1.lon
	return math.sqrt((dLat**2 + dLon**2))*1.113195e5
	
def servo_pwm(vehicle, servo_num, pwm):
    """
    Enable the servo
    """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        servo_num,
        pwm,
        0,
        0,
        0, 0, 0)

    vehicle.send_mavlink(msg)

def send_ned_velocity(vehicle, n, e, d, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
    0b0000111111000111, # type_mask (only speeds enabled)
    0, 0, 0, # x, y, z positions (not used)
    n, e, d, # x, y, z velocity in m/s
    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for i in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    vehicle.flush()

def send_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
    0,       # time_boot_ms (not used)
    0, 0,    # target system, target component
    mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
    0b0000111111000111, # type_mask (only speeds enabled)
    0, 0, 0, # x, y, z positions (not used)
    velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
    0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
    0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)
    """ for i in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(1) """
    vehicle.flush()

def get_location_offset_meters(original_location, dNorth, dEast, alt):
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)

def arm_and_takeoff(vehicle, alt):
    print("Pre-arm Checking ...")

    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise ...")
        time.sleep(5)

    print("Init Finished")
    vehicle.mode = VehicleMode("GUIDED")
    time.sleep(1)
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming ...")
        time.sleep(1)
        vehicle.armed = True
    print("Prepare to Take off !!")
    vehicle.simple_takeoff(alt)

    while True:
        print("Alt : %s" %vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >=alt*0.95:
            print("Reach the alt.")
            break
        time.sleep(1)
		
def security_lock(vehicle,channel):
	'''
	Use the RC channel 8 to start/stop the mission on RaspberryPi, Low : Stop, High : Start
	'''
	while vehicle.channels[channel] > 1500:
		print ("RC_%s :%s" %channel,vehicle.channels[channel])
		print ("switch down to continue the mission")
		time.sleep(1)	
	print ("Start Mission")

def stop_monitor(vehicle):
    # Not success yet
    while True:
        if vehicle.channels['8'] > 1500:
            print("Stop mission ,RTL")
            vehicle.mode = VehicleMode("RTL")
            sys.exit(0)

def rbga(place):

    global wp, sol
    population = 10
    generation = 1000
    numofpoint = len(place)
    
    #check for the start point
    findorigin = np.where(place == 0)
    for m in range(0,len(place)):
        originlen = np.where( m == findorigin[0])
        if len(originlen[0]) > 1:
            origin = m
            break
        else:
            origin = -1
                
    if origin == -1:
        place = np.insert(place,numofpoint,0,axis = 0 ) 
        origin = numofpoint
    
    numofpoint = len(place)
    
    wp = np.zeros((numofpoint,2))
    cost = np.zeros((numofpoint,numofpoint))
    initial = np.zeros((population,numofpoint))
    pathorder = np.zeros((population,numofpoint))
    neworder = np.zeros((population,numofpoint))
    largeorder = np.zeros((population,numofpoint))
    largecost = np.zeros((population,1))
    newcost = np.zeros((population,1))
    stepcost = np.zeros((numofpoint-1,1))
    pathcost = np.zeros((population,1))
    bestcost = np.zeros((generation,1))
    bestorder = np.zeros((generation,numofpoint))
    child = np.zeros((1,numofpoint))

    
    for i in range(0,numofpoint):
        for j in range(0,numofpoint):
            cost[i,j] = math.sqrt(sum((place[i,:]-place[j,:])**2))
        
        
    for i in range(0,population):    
        initial[i,0] = 0
        initial[i,1:] = np.random.permutation(numofpoint-1)+1
    
        pathorder[i] = initial[i]
        for j in range(0,numofpoint-1):
            stepcost[j] = cost[int(pathorder[i,j]),int(pathorder[i,j+1])]
        pathcost[i] = sum(stepcost)    
    
    for i in range(0,1000):
        smallgroup = np.where(pathcost == min(pathcost))
        smallpos = smallgroup[0]
        smallselect = smallpos[0]
    
        neworder[0:int(0.2*population)] = pathorder[smallselect]
        newcost[0:int(0.2*population)] = pathcost[smallselect]
        smallorder = neworder[0:int(0.2*population)]
        smallcost = newcost[0:int(0.2*population)]
    
    # CROSSOVER 0.7
        cross = round(numofpoint*0.7)
        temp = 0
    
        for j in range(0,population):
            if pathcost[j] != smallcost[0]:
                largeorder[temp] = pathorder[j]
                largecost[temp] = pathcost[j]
                temp += 1
            
        cross_randpoint = int(numofpoint-cross + 1)
        
        for k in range(0,int(0.9*population)-int(0.2*population)):
            randlarge = np.random.permutation(int(0.9*population)-int(0.2*population))
            cross_parent_dad = largeorder[randlarge[0]]
            cross_pointselect = np.random.permutation(cross_randpoint)
            randsmall = np.random.permutation(int(0.2*population))
            cross_startpoint = cross_pointselect[0]
            cross_endpoint = int(cross_startpoint + cross - 1)
            cross_parent_mom = smallorder[randsmall[0]]
            cross_part_of_parent = cross_parent_mom[cross_startpoint:(cross_endpoint + 1)]
            s = 0
            for p in range(len(cross_part_of_parent)):
                cross_parent_dad = cross_parent_dad[cross_parent_dad != cross_part_of_parent[p]]
                #largegroup = cross_parent_dad
                
            for q in range(numofpoint):
                if q < cross_startpoint:
                    child[0,q] = cross_parent_dad[q]
                elif q >= cross_startpoint and q <= cross_endpoint :
                    child[0,q] = cross_part_of_parent[s]
                    s += 1
                else :
                    child[0,q] = cross_parent_dad[q-cross_endpoint + cross_startpoint-1]
         
            neworder[int(0.2*population) + k] = child[0,:] 
        
        mut_randpath = np.random.permutation(population)
        mut_rand1 = np.random.permutation(numofpoint)
        mut_rand2 = np.random.permutation(numofpoint)
    
        if i % 10 == 0:
            mut_orderselect1 = np.array(pathorder[mut_randpath[0]])
            mut_orderselect2 = np.array(pathorder[mut_randpath[0]])
            mut_orderselect1[mut_rand1[0]] = mut_orderselect2[mut_rand2[0]]
            mut_orderselect1[mut_rand2[0]] = mut_orderselect2[mut_rand1[0]]
            neworder[population-1] = mut_orderselect1
        else:
            mut_orderselect1 = pathorder[mut_randpath[0]]
            neworder[population-1] = mut_orderselect1
    
        for u in range(0,population):
            for j in range(0,numofpoint-1):
                stepcost[j] = cost[int(neworder[u,j]),int(neworder[u,j+1])]
            newcost[u] = sum(stepcost)
            pathcost[u] = newcost[u]
        
        pathorder = neworder
        pathcost = newcost
        find_cost_min = np.where(pathcost == min(pathcost))
        cost_min = find_cost_min[0]
        bestcost[i] = min(pathcost)
        bestorder[i,:] = pathorder[cost_min[0],:]
    
    endorder = bestorder[i,:]
    if endorder[0] ==  origin:
        endorder = endorder
    else:
        endorder = endorder[::-1]
    np.int8(endorder)
    sol = np.int8(endorder)
    
    for O in range(len(sol)):
        wp[O] = place[sol[O]]
    
    return wp, place, sol
