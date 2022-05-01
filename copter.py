##########DEPENDENCIES#############
from pymavlink import mavutil
from dronekit import connect, VehicleMode,LocationGlobalRelative
import math
import time
import math
from coordinates import PREDEFINED_COORDINATES

#########VARIABLES#################

SLEEP_TIME = 1
SERVO_OPEN_VALUE = 1500
SERVO_CLOSE_VALUE = 1150

#########FUNCTIONS#################

def connectCopter(connection_string):

	if connection_string.lower() == 'sitl':
		import dronekit_sitl
		sitl = dronekit_sitl.start_default()
		connection_string = sitl.connection_string()    

	return connect(connection_string,wait_ready=True, timeout=60, baud=57600)

def get_distance_meters(targetLocation, currentLocation):

	dLat = targetLocation.lat - currentLocation.lat
	dLon = targetLocation.lon - currentLocation.lon
	
	return math.sqrt((dLon*dLon) + (dLat*dLat)) * 1.113195e5

def move(north, east, down, vehicle):

	msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,   
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000111111111000, 
        north, east, down,
        0, 0, 0, 
        0, 0, 0, 
        0, 0)   

	vehicle.send_mavlink(msg)
	vehicle.flush()

#########CLASS DEFINITION##########

class Copter:
	def __init__(self, name, connection_string):
		self.name = name
		self.connection_string = connection_string
		self.takeoff_alt = 0
		self.vehicle = connectCopter(self.connection_string)

	def arm_and_takeoff(self, takeoff_altitude):

		if not takeoff_altitude or takeoff_altitude < 2:
			print("Takeoff altitude should be greater than 2m!")
			exit()

		if takeoff_altitude > 15:
			print("Please enter a takeoff value between 2m - 15m!")	
			exit()

		self.takeoff_alt = takeoff_altitude

		while self.vehicle.is_armable != True:
			print(f"Waiting for {self.name} to become armable.")
			time.sleep(SLEEP_TIME)
		print(f"{self.name} is now armable")

		self.change_mode(mode="GUIDED")

		self.vehicle.armed = True

		while self.vehicle.armed == False:
			print(f"Waiting for {self.name} to become armed.")
			time.sleep(SLEEP_TIME)
		print("props are spinning!!")

		self.vehicle.simple_takeoff(takeoff_altitude)

		while True:
			print(f"Current Altitude: {self.vehicle.location.global_relative_frame.alt}")
			if self.vehicle.location.global_relative_frame.alt >= .95 * takeoff_altitude:
				break
			time.sleep(SLEEP_TIME)
		print("Target altitude reached!!")

	def goto_waypoint(self, drowning_point):

		if not any(drowning_point in coordinate["point"] for coordinate in PREDEFINED_COORDINATES):
			print("Please provide a valid waypoint!!!")
			exit()

		drowning_location = next((coordinate["location"] for coordinate in PREDEFINED_COORDINATES if coordinate["point"] == drowning_point), None)
		target_waypoint = LocationGlobalRelative(drowning_location["latitude"], drowning_location["longitude"], self.takeoff_alt)

		distanceToTargetLocation = get_distance_meters(target_waypoint, self.vehicle.location.global_relative_frame)

		self.vehicle.simple_goto(target_waypoint)

		while self.vehicle.mode.name == "GUIDED":
			currentDistance = get_distance_meters(target_waypoint, self.vehicle.location.global_relative_frame)
			
			if currentDistance < distanceToTargetLocation * .03:
				print("Reached target waypoint.")
				break

	def move_down(self, meters):

		distance_altutide = self.vehicle.location.global_relative_frame.alt - meters

		move(0 ,0, meters, self.vehicle)

		while True:
			print(f"Current Altitude: {self.vehicle.location.global_relative_frame.alt}")

			if self.vehicle.location.global_relative_frame.alt <= 1.05 * distance_altutide:
				break
			time.sleep(SLEEP_TIME)

		print("Dropping altitude reached!!")

	def move_up(self, meters):

		distance_altutide = self.vehicle.location.global_relative_frame.alt + meters
		meters_in_negotive = 0 - meters

		move(0 ,0, meters_in_negotive, self.vehicle)

		while True:
			print(f"Current Altitude: {self.vehicle.location.global_relative_frame.alt}")

			if self.vehicle.location.global_relative_frame.alt >= .95 * distance_altutide:
				break
			time.sleep(SLEEP_TIME)

		print("Move up altitude reached!!")

	def dropper_open(self):

		msg = self.vehicle.message_factory.command_long_encode(
		0, 0,    
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
		0,
		10,    
		SERVO_OPEN_VALUE,          
		0, 0, 0, 0, 0)    
	
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
		time.sleep(SLEEP_TIME)
		print("Dropped.")	

	def dropper_close(self):

		msg = self.vehicle.message_factory.command_long_encode(
		0, 0,    
		mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
		0,
		10,    
		SERVO_CLOSE_VALUE,          
		0, 0, 0, 0, 0)    
	
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
		time.sleep(SLEEP_TIME)
		print("Closed.")

	def change_mode(self, mode):

		self.vehicle.mode = VehicleMode(mode)

		while self.vehicle.mode != mode:
			print(f"Waiting for {self.name} to enter {mode} mode")
			time.sleep(SLEEP_TIME)
		print(f"{self.name} in {mode} mode")

	def waiting_for_disarm(self):
		
		while self.vehicle.armed == True:
			print(f"Waiting for {self.name} to become disarmed.")
			time.sleep(SLEEP_TIME)

		self.vehicle.close()

	def attribute_fetcher(self):

		#Version and attributes
		self.vehicle.wait_ready('autopilot_version')
		print(f'Autopilot version: {self.vehicle.version}')

		#Does the firmware support the companion pc to set the attitude
		print(f'Supports set attitude from companion: {self.vehicle.capabilities.set_attitude_target_local_ned}')

		#Read actual position
		print(f'Position: {self.vehicle.location.global_relative_frame}')

		#Read the actual attitude roll, pitch, yaw
		print(f'Attitude: {self.vehicle.attitude}')

		#Read the actual velocity (m/s)
		print(f'Velocity: {self.vehicle.velocity}') #NED: North East Down convention

		#When did we receive last heartbeat
		print(f'Last Heartbeat: {self.vehicle.last_heartbeat}')

		#Is the vehicle good to arm
		print(f'Is the {self.name} armable: {self.vehicle.is_armable}')

		#What is total groundspeed
		print(f'Groundspeed: {self.vehicle.groundspeed}')

		#What is the actual flight mode
		print(f'Mode: {self.vehicle.mode.name}') 	  

		#Is the vehicle armed
		print(f'Armed: {self.vehicle.armed}') 	    

		#Is state estimation filter ok
		print(f'EKF Ok: {self.vehicle.ekf_ok}')

	def land(self):
		self.change_mode("LAND")