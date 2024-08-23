import airsim
import time
import math


def move_forward(client, duration, speed):
    current_yaw = get_current_yaw(client)
    # Convert yaw to radians for calculation
    yaw_radians = math.radians(current_yaw)
    # Calculate velocity components based on the drone's orientation
    vx = math.cos(yaw_radians) * speed
    vy = math.sin(yaw_radians) * speed
    # Command to move in the direction of the nose
    #print(f"Moving forward with vx: {vx}, vy: {vy}, yaw: {current_yaw}")
    client.moveByVelocityZAsync(vx=vx, vy=vy, z=client.getMultirotorState().kinematics_estimated.position.z_val, duration=duration).join()

def get_current_yaw(client):
    orientation = client.simGetVehiclePose().orientation
    euler_angles = airsim.to_eularian_angles(orientation)
    yaw = math.degrees(euler_angles[2])
    return yaw
    
def move_diagonally_forward_up(client, duration, forward_speed, upward_speed):
    #print("Moving diagonally forward and up...")
    # Get current yaw and calculate velocity components
    current_yaw = get_current_yaw(client)
    yaw_radians = math.radians(current_yaw)
    vx = math.cos(yaw_radians) * forward_speed
    vy = math.sin(yaw_radians) * forward_speed

    # Calculate the current altitude from which to start the ascent
    current_altitude = client.getMultirotorState().kinematics_estimated.position.z_val
    desired_altitude = current_altitude - 5  # Assuming you want to move up by 5 meters

    # Move forward and upward simultaneously
    client.moveByVelocityZAsync(vx=vx, vy=vy, z=desired_altitude, duration=duration).join()

def move_diagonally_forward_down(client, duration, forward_speed, downward_speed):
    #print("Moving diagonally forward and down...")
    # Get current yaw and calculate velocity components
    current_yaw = get_current_yaw(client)
    yaw_radians = math.radians(current_yaw)
    vx = math.cos(yaw_radians) * forward_speed
    vy = math.sin(yaw_radians) * forward_speed

    # Calculate the current altitude from which to start the descent
    current_altitude = client.getMultirotorState().kinematics_estimated.position.z_val
    desired_altitude = current_altitude + 5  # Assuming you want to move down by 5 meters

    # Move forward and downward simultaneously
    client.moveByVelocityZAsync(vx=vx, vy=vy, z=desired_altitude, duration=duration).join()


def move_diagonally_down_yaw(client, yaw_change, duration, forward_speed, downward_speed):
    # Rotate to the new yaw
    current_yaw = get_current_yaw(client)
    target_yaw = current_yaw + yaw_change
    client.rotateToYawAsync(target_yaw, timeout_sec=duration).join()
    # Calculate diagonal movement velocities
    vx = forward_speed * math.cos(math.radians(target_yaw))
    vy = forward_speed * math.sin(math.radians(target_yaw))
    z = client.getMultirotorState().kinematics_estimated.position.z_val + downward_speed
    # Move diagonally down with the new yaw
    client.moveByVelocityZAsync(vx=vx, vy=vy, z=z, duration=duration).join()
    #print(f"Moved {yaw_change} degrees to {'right' if yaw_change > 0 else 'left'} and down")

def move_45_degrees_right_down(client, duration, speed):
    move_diagonally_down_yaw(client, 45, duration, speed, 5)

def move_45_degrees_left_down(client, duration, speed):
    move_diagonally_down_yaw(client, -45, duration, speed, 5)
    
def move_diagonally_up_yaw(client, yaw_change, duration, forward_speed, upward_speed):
    # Rotate to the new yaw
    current_yaw = get_current_yaw(client)
    target_yaw = current_yaw + yaw_change
    client.rotateToYawAsync(target_yaw, timeout_sec=duration).join()
    # Calculate diagonal movement velocities
    vx = forward_speed * math.cos(math.radians(target_yaw))
    vy = forward_speed * math.sin(math.radians(target_yaw))
    z = client.getMultirotorState().kinematics_estimated.position.z_val - upward_speed
    # Move diagonally up with the new yaw
    client.moveByVelocityZAsync(vx=vx, vy=vy, z=z, duration=duration).join()
    #print(f"Moved {yaw_change} degrees to {'right' if yaw_change > 0 else 'left'} and up")

def move_45_degrees_right_up(client, duration, speed):
    move_diagonally_up_yaw(client, 45, duration, speed, 5)

def move_45_degrees_left_up(client, duration, speed):
    move_diagonally_up_yaw(client, -45, duration, speed, 5)
    
    
def rotate_by_degrees(client, yaw_change, duration):
    # Get the current yaw and calculate the target yaw
    current_yaw = get_current_yaw(client)
    target_yaw = current_yaw + yaw_change
   # print(f"Rotating from {current_yaw:.2f} degrees to {target_yaw:.2f} degrees.")
    # Rotate to the new yaw
    client.rotateToYawAsync(target_yaw, timeout_sec=duration).join()

def rotate_45_degrees_right(client, duration):
    #print("Turning right by 45 degrees...")
    rotate_by_degrees(client, 45, duration)  # Rotate 45 degrees to the right

def rotate_45_degrees_left(client, duration):
    #print("Turning left by 45 degrees...")
    rotate_by_degrees(client, -45, duration)  # Rotate 45 degrees to the left