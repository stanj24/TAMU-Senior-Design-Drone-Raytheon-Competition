from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600, wait_ready=True)
#921600 is the baudrate that you have set in the mission plannar or qgc

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)
#Function for the position
def send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration=0):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
      0,  # time_boot_ms (not used)
      0, 0,  # target system, target component
      mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame Needs to be MAV_FRAME_BODY_NED for forward/back left/right control.
      0b0000111111000111,  # type_mask
      0, 0, 0,  # x, y, z positions (not used)
      velocity_x, velocity_y, velocity_z,  # m/s
      0, 0, 0,  # x, y, z acceleration
      0, 0)
    for x in range(0, duration):
      vehicle.send_mavlink(msg)
      time.sleep(1)

# Initialize the takeoff sequence to a position of 20 meters
arm_and_takeoff(20)

print("Take off complete")

# Hover for 10 seconds
time.sleep(15)
# This is the command to move the copter 10 m/s forward for 15 sec.
velocity_x = 10
velocity_y = 10
velocity_z = 0
duration = 15
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

# backwards at 10 m/s for 10 sec.
velocity_x = -10
velocity_y = -10
velocity_z = 0
duration = 15
send_body_ned_velocity(velocity_x, velocity_y, velocity_z, duration)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()
