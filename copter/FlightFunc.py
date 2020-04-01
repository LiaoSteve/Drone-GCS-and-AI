import time, sys
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Command
from pymavlink import mavutil # Needed for command message definitions
import os
import math, json
from math import asin,sin,cos,sqrt,radians

def read_json(path):
    # READ COORDINATES FROM JSON 
    input_file = open(path)
    json_array = json.load(input_file)
    coordinates = json_array['features'][0]['geometry']['coordinates']
    return coordinates  
 
def haversine(pos1,pos2_lon,pos2_lat):
    """
    Get the distance between two positions (note that inputs are in decimal degree)
    pos1 : current point
    pos2 : Reference point 
    """
    lon1, lat1, lon2, lat2 = map(radians, [pos1.lon, pos1.lat, pos2_lon, pos2_lat])
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * asin(sqrt(a)) 
    r = 6378137.0 #-- Radius of "spherical" earth in meter
    return c * r   
	
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

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
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

def get_attributes(vehicle):
    print (vehicle.location.local_frame)    #NED
    print (vehicle.attitude)
    print ("Velocity: %s" % vehicle.velocity)
    print (vehicle.gps_0)
    print ("Groundspeed: %s" % vehicle.groundspeed)
    print ("Airspeed: %s" % vehicle.airspeed)
    print ( vehicle.gimbal)
    print (vehicle.battery)
    print ("EKF OK?: %s" % vehicle.ekf_ok)
    print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
    print (vehicle.rangefinder)
    print ("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print ("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
    print ("Heading: %s" % vehicle.heading)
    print ("Is Armable?: %s" % vehicle.is_armable)
    print ("System status: %s" % vehicle.system_status.state)  
