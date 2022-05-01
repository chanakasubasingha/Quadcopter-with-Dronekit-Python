from copter import Copter

# /dev/ttyUSB0, tcp:0.0.0.0:1998, SITL
quadcopter = Copter(name = "Quadcopter", connection_string = 'sitl') 
quadcopter.arm_and_takeoff(takeoff_altitude = 2)
quadcopter.land()
quadcopter.waiting_for_disarm()
