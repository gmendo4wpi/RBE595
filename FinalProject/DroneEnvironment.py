import gym
import numpy as np
import airsim
import cv2
import time
from movements import move_forward, move_45_degrees_right_up, move_45_degrees_left_up, move_45_degrees_right_down, move_45_degrees_left_down, rotate_45_degrees_right, rotate_45_degrees_left

class DroneEnv(gym.Env):
    """Custom Environment that follows gym interface."""
    metadata = {'render.modes': ['console']}

    def __init__(self):
        super(DroneEnv, self).__init__()
        # Define action and observation space
        self.action_space = gym.spaces.Discrete(7)  # Expanded to include new actions
        self.observation_space = gym.spaces.Box(low=0, high=255, shape=(72, 128, 1), dtype=np.uint8)

        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.target_x = 35
        self.start_time = None
        self.num_actions = 0
        self.time_limit = 60  # seconds
        self.action_limit = 25
        self.previous_x_val = None  # To store previous x position

    def step(self, action):
        if self.start_time is not None and (time.time() - self.start_time > self.time_limit or self.num_actions >= self.action_limit):
            return self.reset(), -100, True, {'timeout': True}  # Resetting with a timeout or action limit reached
        self.num_actions += 1

        # Store previous position for movement calculation
        previous_position = self.client.simGetVehiclePose().position

        # Action handling
        if action == 0:
            move_forward(self.client, 1, 2.5)  # Move forward
        elif action == 1:
            move_45_degrees_right_up(self.client, 2, 5)  # Move diagonally up to the right
        elif action == 2:
            move_45_degrees_left_up(self.client, 2, 5)  # Move diagonally up to the left
        elif action == 3:
            move_45_degrees_right_down(self.client, 2, 2)  # Move diagonally down to the right
        elif action == 4:
            move_45_degrees_left_down(self.client, 2, 2)  # Move diagonally down to the left
        elif action == 5:
            rotate_45_degrees_right(self.client, 1)  # Rotate 45 degrees to the right
        elif action == 6:
            rotate_45_degrees_left(self.client, 1)  # Rotate 45 degrees to the left

        # Get the new depth image and check distances
        depth_image = self.get_depth_image()
        front_distance = np.min(depth_image) if depth_image is not None else float('inf')

        # Get current position to check altitude and lateral movements
        current_position = self.client.simGetVehiclePose().position
        self.positions.append((current_position.x_val, current_position.y_val, current_position.z_val))

        # Calculate x and z changes
        x_change = current_position.x_val - previous_position.x_val
        z_change = current_position.z_val - previous_position.z_val

        # Conditions for completing the episode
        done = bool(front_distance < 0.5 or current_position.x_val >= self.target_x)

        # Check if the drone has strayed too far sideways
        if abs(current_position.z_val) > 10:
            reward = -25  # Heavy penalty for straying too far laterally
            done = True
        else:
            # Default step cost
            reward = -1
            # Adjust reward based on vertical movement
            if z_change > 0:
                reward += 5 * z_change  # Reward for ascending
            if z_change < 0:
                reward -= 3 * abs(z_change)  # Penalty for descending

            # Encourage forward movement and penalize backward movement
            if x_change > 0:
                reward += 7 * x_change  # Amplified reward for moving forward
            if x_change < 0:
                reward += x_change  # Negative reward for moving away from the goal

            if front_distance < 1.0:
                reward -= (1.0 / front_distance) * 5  # Scale the penalty by distance
            if front_distance < 0.5:
                reward = -5  # Large penalty for near collision
            elif current_position.x_val >= self.target_x:
                reward = 100  # Large reward for reaching the goal

        return np.array(depth_image), reward, done, {}


    def get_depth_image(self):
        """Retrieve and process depth image from the drone's sensor."""
        responses = self.client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, pixels_as_float=True)])
        if responses:
            response = responses[0]
            img_data = np.array(response.image_data_float, dtype=np.float32)
            depth_image = cv2.resize(img_data, (128, 72))
            depth_image = np.expand_dims(depth_image, axis=-1)
        return depth_image

    def reset(self):
        self.start_time = time.time()
        self.num_actions = 0
        self.previous_x_val = None  # Reset previous x position
        self.client.reset()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        self.client.moveToZAsync(-5, 1).join()
        self.positions = []  # Clear the position log
        return self.get_depth_image()
        

    def close(self):
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
