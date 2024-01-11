##########DEPENDENCIES#############

from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException,Command,LocationGlobal
import time
import socket
#import exceptions
import math
import argparse
from pymavlink import mavutil

#########FUNCTIONS#################

#connection_string='/dev/ttyTHS1'
#connection_string= 'tcp:127.0.0.1:5763'
#baud=57600

def connectMyCopter():

	parser = argparse.ArgumentParser(description='commands')
	parser.add_argument('--connect')
	args = parser.parse_args()

	connection_string = args.connect


	vehicle = connect(connection_string,baud=57600,wait_ready=True)

	return vehicle

def arm():
	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Vehicle is now armed.")

	return None

def disconnect_rover():
    vehicle.close()

def get_version():
    global vehicle
    return vehicle.version
def get_location():
    global vehicle
    return vehicle.location.global_frame
    
def get_velocity():
    global vehicle
    return vehicle.velocity
def get_battery_info():
    global vehicle
    return vehicle.battery
def get_mode():
    global vehicle
    return vehicle.mode.name
def get_home_location():
    global vehicle
    return vehicle.home_location
def get_heading():
    global vehicle
    return vehicle.heading

def get_EKF_status():
    return vehicle.ekf_ok

def get_ground_speed():
    return vehicle.groundspeed

#def roverinfo():
    #yukarıdaki bilgiler alınarak topluca yollancaktır

def read_channel(channel):

    return vehicle.channels[str(channel)]


def set_groundspeed(speed):
    global vehicle
    print("groundspeed set to: " % speed)
    vehicle.groundspeed = speed

def set_channel(channel, value):
    global vehicle
    vehicle.channels.overrides[channel] = value
def return_to_launch_location():
    #carefull with using this! It wont detect obstacles!
    vehicle.mode = VehicleMode("RTL")

def disarm():
        vehicle.armed = False
        while vehicle.armed== True:
                print("Waiting for vehicle to become disarmed.")
                time.sleep(1)
        print("Vehicle is now disarmed.")

        return None

def goto(targetLocation):
	distanceToTargetLocation = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)

	vehicle.simple_goto(targetLocation)

	while vehicle.mode.name=="GUIDED":
		currentDistance = get_distance_meters(targetLocation,vehicle.location.global_relative_frame)
		if currentDistance<distanceToTargetLocation*.05:
			print("Reached target waypoint.")
			time.sleep(2)
			break
		time.sleep(1)
	return None

##Send a velocity command with +x being the heading of the drone.
def send_local_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, #frame
		0b0000111111000111, #type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
		0, 0, 0, #x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

##Send a velocity command with +x being the heading of the drone.
def send_global_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, #frame
		0b0000111111000111, #type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
		0, 0, 0, #x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)    

def get_distance_meters(targetLocation,currentLocation):
	dLat=targetLocation.lat - currentLocation.lat
	dLon=targetLocation.lon - currentLocation.lon
	
	return math.sqrt((dLon*dLon)+(dLat*dLat))*1.113195e5


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation;
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print(" Upload new commands to vehicle")
    cmds.upload()


##########MAIN EXECUTABLE###########


vehicle = connectMyCopter()

#vehicle.wait_ready('autopilot_version')
#print('Autopilot version: %s'%vehicle.version)
#ekf = get_EKF_status()
#print('Rover EKF: %s'%ekf)

arm()

#adds_square_mission(vehicle.location.global_frame,20)

#return_to_launch_location()

#send_local_ned_velocity(1,0,0) 
#print("Moving forward at 1 m/s with local NED")
#time.sleep(5)
#send_local_ned_velocity(1,1,0)
#print("Turning to the right")
#time.sleep(2)
#send_local_ned_velocity(1,-1,0)
#print("Turning to the left")
#time.sleep(2)

#send_global_velocity(1,0,0,3)
#print("Moving TRUE NORTH")
#time.sleep(3)
#send_global_velocity(-1,0,0,3)
#print("MOVING TRUE SOUTH")
#time.sleep(3)
#send_global_velocity(0,1,0,3)
#print("MOVING TRUE EAST")
#time.sleep(3)
#send_global_velocity(0,-1,0,3)
#print("MOVING TRUE WEST")
#time.sleep(3)

#point2 = LocationGlobalRelative(-35.363244, 149.168801,1)
#vehicle.simple_goto(point2, groundspeed=10)
#goto(point2)
#time.sleep(4)
#disarm()
