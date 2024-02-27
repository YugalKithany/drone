# ready to run example: PythonClient/multirotor/hello_drone.py
# note: async methods take a long time to execute, add join() to wait for it finish 
# NED coordinates: +x:forward +y:right +z:down

import airsim
import os
import time
import numpy as np
import cv2
import math

from gen_traj import Generate
from perception import Perception
lead = "Drone_L"
chase = "Drone_C"

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
# client.reset()
curr_state = client.simGetVehiclePose(lead) # print("lead state", curr_state)

#### Initalize Lead and Chaser Drones
client.enableApiControl(True,lead)
client.armDisarm(True, lead)
client.takeoffAsync(10, lead).join()

client.enableApiControl(True,chase)
client.armDisarm(True, chase)
client.takeoffAsync(10, chase).join()


# Lead Drone Movement in Figure 8
def xyz(args, real_t):
  period, sizex, sizey = args
  print('period/sizex/sizey: ', period, sizex, sizey)
  if not period:
    period = real_t[-1]
  t = real_t / period * 2 * np.pi
  x = np.sqrt(2) * np.cos(t) / (1 + np.sin(t) ** 2)
  y = x * np.sin(t)
  x = sizex * x
  y = sizey * y
  z = np.ones_like(x) * 1.5
  return x, y, z


# Initialize empty arrays to store positions
lead_positions = []
chase_positions = []

# Game loop.
count = 0
while True:
    dt_move = .1
    vel=1
    x,y,z = xyz([100,100,100],count)

    # client.moveByVelocityAsync(1,0,0,dt_move,vehicle_name=lead).join()
    # client.moveToPositionAsync(x, y, curr_state.position.z_val, vel, vehicle_name=lead)
    client.moveToPositionAsync(x, y, curr_state.position.z_val, vel, vehicle_name=lead)

    # identify and store location of lead
    lead_pose = [client.simGetVehiclePose(lead).position.x_val,
                 client.simGetVehiclePose(lead).position.y_val,
                 client.simGetVehiclePose(lead).position.z_val]
    print("Lead position",lead_pose)
    lead_positions.append(lead_pose)

    # identify and store location of chase
    curr_pose_chase = [client.simGetVehiclePose(chase).position.x_val,
                       client.simGetVehiclePose(chase).position.y_val,
                       client.simGetVehiclePose(chase).position.z_val]
    chase_positions.append(curr_pose_chase)

    # curr_pose_rel = [lead_pose[0]-curr_pose_chase[0], lead_pose[1]-curr_pose_chase[1], lead_pose[2]-curr_pose_chase[2]]
    # print("relative pose",curr_pose_rel)
    count += 1

    client.moveToPositionAsync(lead_pose[0], lead_pose[1], lead_pose[2], vel, vehicle_name=chase)

    k=10
    vx = k * (lead_pose[0]-curr_pose_chase[0])
    vy = k * (lead_pose[1]-curr_pose_chase[1])

    # Move chase drone by velocity
    client.moveByVelocityZAsync(vx, vy, client.simGetVehiclePose(chase).position.z_val, 5, dt_move,vehicle_name=chase)
    # client.moveByVelocityZAsync(vx, vy, curr_pose_chase[2], 1, airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0), vehicle_name=chase).join()




    time.sleep(0.1)
    if count == 1000:
        break

print("Finished")
print(lead_positions)
print("Finished")
print(chase_positions)
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)


# write up how time works in the while loop (.join())

# given xl vl and some offset p#




#### Addendum::


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