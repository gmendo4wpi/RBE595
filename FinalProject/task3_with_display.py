import airsim
import numpy as np
import time

def get_depth_image(client):
    """Retrieve and process depth image from the drone's sensor."""
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, pixels_as_float=True)])
    if responses:
        response = responses[0]
        img_data = np.array(response.image_data_float, dtype=np.float32)
        depth_image = np.reshape(img_data, (response.height, response.width))
        return depth_image
    else:
        return None

def main():
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    print("Taking off...")
    client.takeoffAsync().join()
    client.moveToZAsync(-5, 5).join()  # Adjust altitude to a safe level

    target_x = 65
    current_x = 0
    step = 5
    obstacle_threshold = 2.5  # meters

    while current_x < target_x:
        print("Moving forward...")
        client.moveByVelocityAsync(2.5, 0, 0, 1).join()  # Move forward at 5 m/s
        depth_image = get_depth_image(client)
        if depth_image is not None:
            front_distance = np.min(depth_image)  # Simple obstacle detection in front
            print("Detected distance:", front_distance)

            while front_distance < obstacle_threshold:
                print("Obstacle detected! Ascending...")
                client.moveToZAsync(-15, 2).join()  # Move higher
                depth_image = get_depth_image(client)
                front_distance = np.min(depth_image) if depth_image is not None else float('inf')

        current_x += step  # Simulate forward movement increment

    print("Destination reached. Landing...")
    client.moveToPositionAsync(65, 0, -10, 5).join()  # Move to the exact landing spot
    client.landAsync().join()

    client.armDisarm(False)
    client.enableApiControl(False)
    print("Mission completed successfully.")

if __name__ == "__main__":
    main()
