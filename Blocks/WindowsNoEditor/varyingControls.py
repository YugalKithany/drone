
### Pseudocode:
'''
  Import lib, connect to AirSim simulation, initalize/take off drones
  Specify Vector3r coords path for lead drone in 'thread1'.
  In Main:
    Start thread1   
    Game Loop:
      Set inital parameters
      Lead drone will be updated by other thread, just need to save new coords in array.
      Calculate Chase drone new coords using methods (given coords of lead, or velocity estimation, or ??)
      Move Chase drone, save new coords in array. 
      Once loop iterates for x times, break

      Use helper function 'calculate' to run analytics on chaser drone following abilities. 
      Disarm Drones, and turn off API connection
'''

# TODO: 
''' write up how time works in the while loop (.join())
    given xl vl and some offset vector p

    √ Save data within python, pickle???
    moveToPath for lead drone -- with multithreading
    velocity control for the chaser drone (PID?)
    Draw a block diagram, write down how time works. Make the code starting to match the block diagram for code execution
    √ Can we decouple while loop. Ex. Lead and Chaser running with different while loops -> Used multithreading. 
    √ Control how much time each module runs. ->  Used a timeout time for moveToPath
    √ Architect the simulation loop. 

'''

# ready to run example: PythonClient/multirotor/hello_drone.py
# note: async methods take a long time to execute, add join() to wait for it finish 
# NED coordinates: +x:forward +y:right +z:down

import airsim
import os
import time
import numpy as np
# import cv2
import math
import matplotlib.pyplot as plt
import pickle
import threading

# Copy over files from algo folder
# from gen_traj import Generate
# from perception import Perception
lead = "Drone_L"
chase = "Drone_C"

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
curr_state = client.simGetVehiclePose(lead) # print("lead state", curr_state)

# Initalize Lead and Chaser Drones
client.enableApiControl(True,lead)
client.armDisarm(True, lead)
client.takeoffAsync(10, lead).join()

client.enableApiControl(True,chase)
client.armDisarm(True, chase)
client.takeoffAsync(10, chase).join()


def calculations(lead, chase):
  data=[lead,chase]

  timestamp = int(time.time())
  filename = f"data_{timestamp}.pickle"
  
  # Open the file in binary write mode
  with open(filename, "wb") as f:
    pickle.dump(data, f) # Pickle the data (list containing arrays)

  lead = np.array(lead[0:len(lead)-1])
  chase = np.array(chase[1:])
  
  lead_x, lead_y, lead_z = np.transpose(lead)
  chase_x, chase_y, chase_z = np.transpose(chase)

  ### LEAD VS CHASE XYZ COORD
  # percent_error_x = np.mean((np.abs(lead_x - chase_x) / np.abs(lead_x)) * 100)
  # percent_error_y = np.mean((np.abs(lead_y - chase_y) / np.abs(lead_y)) * 100)
  # percent_error_z = np.mean((np.abs(lead_z - chase_z) / np.abs(lead_z)) * 100)


  percent_error_x = np.mean((np.abs(np.mean(lead_x) - np.mean(chase_x)) / np.mean(np.abs(lead_x))) * 100)
  percent_error_y = np.mean((np.abs(np.mean(lead_y) - np.mean(chase_y)) / np.mean(np.abs(lead_y))) * 100)
  percent_error_z = np.mean((np.abs(np.mean(lead_z) - np.mean(chase_z)) / np.mean(np.abs(lead_z))) * 100)
  percent_error = (percent_error_x, percent_error_y, percent_error_z)

  fig, axes = plt.subplots(3, 1, figsize=(10, 6))
  axes[0].plot(lead_x, chase_x, marker='o', linestyle='')
  axes[0].set_title('Lead vs. Chase X-coordinates')
  axes[0].set_xlabel('Lead X')
  axes[0].set_ylabel('Chase X')
  axes[0].grid(True)
  # Plot y-coordinates
  axes[1].plot(lead_y, chase_y, marker='o', linestyle='')
  axes[1].set_title('Lead vs. Chase Y-coordinates')
  axes[1].set_xlabel('Lead Y')
  axes[1].set_ylabel('Chase Y')
  axes[1].grid(True)
  # Plot z-coordinates
  axes[2].plot(lead_z, chase_z, marker='o', linestyle='')
  axes[2].set_title('Lead vs. Chase Z-coordinates')
  axes[2].set_xlabel('Lead Z')
  axes[2].set_ylabel('Chase Z')
  axes[2].grid(True)
  plt.tight_layout()
  plt.show()

  ### XYZ COORD VS INDEX
  fig, axes = plt.subplots(3, 1, figsize=(10, 6))
  # Plot x-coordinates with array index
  axes[0].plot(range(len(lead_x)), lead_x, label='Lead X')
  axes[0].plot(range(len(chase_x)), chase_x, label='Chase X')
  axes[0].set_title('Lead vs. Chase X-coordinates (Array Index)')
  axes[0].set_xlabel('Array Index')
  axes[0].set_ylabel('X-coordinate')
  axes[0].legend()
  axes[0].grid(True)
  # Plot y-coordinates with array index
  axes[1].plot(range(len(lead_y)), lead_y, label='Lead Y')
  axes[1].plot(range(len(chase_y)), chase_y, label='Chase Y')
  axes[1].set_title('Lead vs. Chase Y-coordinates (Array Index)')
  axes[1].set_xlabel('Array Index')
  axes[1].set_ylabel('Y-coordinate')
  axes[1].legend()
  axes[1].grid(True)
  # Plot z-coordinates with array index
  axes[2].plot(range(len(lead_z)), lead_z, label='Lead Z')
  axes[2].plot(range(len(chase_z)), chase_z, label='Chase Z')
  axes[2].set_title('Lead vs. Chase Z-coordinates (Array Index)')
  axes[2].set_xlabel('Array Index')
  axes[2].set_ylabel('Z-coordinate')
  axes[2].legend()
  axes[2].grid(True)
  plt.tight_layout()
  plt.show()

  return percent_error 





## Make waypoints instead of move on path

waypoints = [
    airsim.Vector3r(120, 0, 5),  # Start point
    airsim.Vector3r(120, -125, 5),  # Move right
    airsim.Vector3r(0, -125, 5),  # Move up and right
    airsim.Vector3r(0, 5, 5),  # Move up
    airsim.Vector3r(0, 0, 5),  # Move back to start point (completes rectangle)
]
path_velocity = airsim.Vector3r(2, 2, 0) 

# Initialize empty arrays to store positions
lead_positions = []
chase_positions = []
# Game loop
count = 0

z=33
result = client.moveOnPathAsync([airsim.Vector3r(0,0,z),
                                airsim.Vector3r(0,15,z),
                                airsim.Vector3r(0,-15,z),
                                airsim.Vector3r(0,0,z),
                                airsim.Vector3r(10,0,z),
                                airsim.Vector3r(10,-5,z),
                                airsim.Vector3r(0,0,z)

                                # airsim.Vector3r(0,-10,z),
                                # airsim.Vector3r(0,0,z),
                                # ], 1, 5, vehicle_name=lead)

                                ], 3, 20 ,airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1, vehicle_name=lead)






while True:
    dt_move = .1
    vel=3
    # x,y,z = xyz([100,10,10],count)

    # client.moveToPositionAsync(x, y, curr_state.position.z_val, vel, vehicle_name=lead)
    # client.moveToPositionAsync(x, y, curr_state.position.z_val, vel, vehicle_name=lead)

    # identify and store location of lead
    lead_pose = [client.simGetVehiclePose(lead).position.x_val,
                 client.simGetVehiclePose(lead).position.y_val,
                 client.simGetVehiclePose(lead).position.z_val]    # print("Lead position",lead_pose)
    lead_positions.append(lead_pose)

    client.moveToPositionAsync(lead_pose[0], lead_pose[1], lead_pose[2], vel, vehicle_name=chase)
    # identify and store location of chase
    curr_pose_chase = [client.simGetVehiclePose(chase).position.x_val,
                       client.simGetVehiclePose(chase).position.y_val,
                       client.simGetVehiclePose(chase).position.z_val]
    chase_positions.append(curr_pose_chase)
    # curr_pose_rel = [lead_pose[0]-curr_pose_chase[0], lead_pose[1]-curr_pose_chase[1], lead_pose[2]-curr_pose_chase[2]]
    # print("relative pose",curr_pose_rel)

    # Move chase drone by velocity
    # k=10
    # vx = k * (lead_pose[0]-curr_pose_chase[0])
    # vy = k * (lead_pose[1]-curr_pose_chase[1])
    # client.moveByVelocityZAsync(vx, vy, client.simGetVehiclePose(chase).position.z_val, 5, dt_move,vehicle_name=chase)
      # client.moveByVelocityZAsync(vx, vy, curr_pose_chase[2], 1, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), vehicle_name=chase).join()
    count += 1

    time.sleep(.1)
    if count == 100:
        break

print("Finished")
ret=calculations(lead_positions, chase_positions)
print(ret)
time.sleep(10)
client.armDisarm(False)
client.enableApiControl(False)



#### XXX: Addendum::
# if __name__ == "__main__":
#   import matplotlib.pyplot as plt
#   from mpl_toolkits.mplot3d import Axes3D
#   x,y,z = xyz([10,1,1],)
#   # Create a figure and 3D axes
#   fig = plt.figure()
#   ax = fig.add_subplot(111, projection='3d')
#   # Plot a 3D scatter plot
#   ax.scatter(x, y, z, c='r', marker='o')
#   # Set labels for the axes
#   ax.set_xlabel('X-axis')
#   ax.set_ylabel('Y-axis')
#   ax.set_zlabel('Z-axis')
#   # Set a title for the plot
#   ax.set_title('3D Scatter Plot')
#   # Show the plot
#   plt.show()



# Take picture ##
# vision = Perception(client)
# img_rgb = vision.capture_RGB(client)
# cv2.imshow("pic",img_rgb)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

# img_segment = vision.capture_segment(client)
# cv2.imshow("pic",img_segment)
# cv2.waitKey(0)
# cv2.destroyAllWindows()



'''
  Import lib, connect to AirSim simulation, initalize/take off drones
  Game Loop:
    Set inital parameters, obtain updated coords of lead drone using helper fxn 'xyz'
    Move Lead drone, save new coords in array.
    Calculate Chase drone new coords using methods (given coords of lead, or velocity estimation, or ??)
    Move Chase drone, save new coords in array. 
    Once loop iterates for x times, break
    Disarm Drones, and turn off API connection
'''



'''
# Lead Drone Movement in Figure 8
def xyz(args, real_t):
  period, sizex, sizey = args
  # print('period/sizex/sizey: ', period, sizex, sizey)
  if not period:
    period = real_t[-1]
  t = real_t / period * 2 * np.pi
  x = np.sqrt(2) * np.cos(t) / (1 + np.sin(t) ** 2)
  y = x * np.sin(t)
  x = sizex * x
  y = sizey * y
  z = np.ones_like(x) * 1.5
  return x, y, z


'''

''' XXX: Design choices:
    Either make large array of waypoints and duplicate paths, or use multithreading to iterate through waypoints repeatedly 

'''
