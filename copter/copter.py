

def goto(target, gotoFunction = vehicle.simple_goto):
    currentLocation = vehicle.location.global_relative_frame
    targetDistance = haversine(currentLocation,target.lon,target.lat)

    print("[ targetDistance ]:",targetDistance," (m)")
    gotoFunction(location=target ,groundspeed=None)
    
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
                
        cur_pos = send_current_mark()
        send_drone_status_to_GCS()
        remainingDistance = haversine(cur_pos,target.lon,target.lat)
        
        vehicle.groundspeed = 1
        #--  error --
        torlerance = 0.08
        if remainingDistance<0.9:
            torlerance = 1
        if remainingDistance<0.31:
            torlerance = 5
        torlerance_dist = remainingDistance * torlerance                                  
      
        print("[Dist to target]:{:.2f}(m),[Speed]:{:.2f}(m/s),[tolerance dist]:{:.2f}(m)".format(remainingDistance,vehicle.groundspeed,torlerance_dist))    

        if remainingDistance<=torlerance_dist: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(0.2) 

def COPTER_JOB():   
    global vehicle    
    try:
        arm_and_takeoff(aTargetAltitude=8)        
        for i in range(len(waypoints)): 
            print("------------------------------------------------------")      
            print('Go to waypoints:',i+1)
            print("[lon,lat]: [",waypoints[i][0],waypoints[i][1],"]")  #[lon ,lat]) 
            print("------------------------------------------------------")     
            target = LocationGlobalRelative(lat=waypoints[i][1],lon=waypoints[i][0],alt=8)
            time.sleep(1)   
            goto(target)  
        print("Mission Completed")
        #print("Setting LAND mode...")
        #vehicle.mode = VehicleMode("LAND")
        print("Setting RTL mode...")
        vehicle.groundspeed = 1
        vehicle.mode = VehicleMode("RTL")                           
        print('TRASH_NUM:',TRASH_NUM)
        print('CAP_NUM:',CAP_NUM)
        while(1):            
            #time.sleep(1)
            cur_pos = send_current_mark()
            send_drone_status_to_GCS()
            print('===================== UAV alive ======================')
            time.sleep(1)
            
        print("Close vehicle object")
        vehicle.close()     
    except Exception as e: 
        print("exception:",e)
        vehicle.close() 
        print(e)     
    except KeyboardInterrupt:
        print('\n\nKeyboardInterrupt')
        print('vehicle.close() and sys.exit()')
        vehicle.close()
        sys.exit()
    finally :
        print('vehicle.close() and sys.exit()')
        vehicle.close() 
        sys.exit()

