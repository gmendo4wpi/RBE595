import matplotlib.pyplot as plt
from stable_baselines3 import DQN
from stable_baselines3.common.evaluation import evaluate_policy
from DroneEnvironment import DroneEnv
import numpy as np


def plot_rewards(rewards):
    plt.figure(figsize=(10, 5))
    plt.plot(rewards, label='Rewards per Episode')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.title('Training Rewards Over Episodes')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_trajectories(positions):
    plt.figure(figsize=(14, 7))
    ax_x = plt.subplot(1, 3, 1)
    ax_y = plt.subplot(1, 3, 2)
    ax_z = plt.subplot(1, 3, 3)
    
    for pos in positions:
        x_vals, y_vals, z_vals = zip(*pos)
        ax_x.plot(x_vals, label='X Position')
        ax_y.plot(y_vals, label='Y Position')
        ax_z.plot(z_vals, label='Z Position')
    
    ax_x.set_title('X Trajectories')
    ax_x.set_xlabel('Timestep')
    ax_x.set_ylabel('X Position')
    ax_x.legend(loc='upper left')

    ax_y.set_title('Y Trajectories')
    ax_y.set_xlabel('Timestep')
    ax_y.set_ylabel('Y Position')
    ax_y.legend(loc='upper left')

    ax_z.set_title('Z Trajectories')
    ax_z.set_xlabel('Timestep')
    ax_z.set_ylabel('Z Position')
    ax_z.legend(loc='upper left')

    plt.tight_layout()
    plt.show()
    
def main():
    env = DroneEnv()
    model = DQN("CnnPolicy", env, verbose=1, buffer_size=10000, learning_starts=1000)

    # Start training
    model.learn(total_timesteps=250)

    # Log rewards and positions for evaluation
    reward_log = []
    all_positions = []
    for i in range(10):  # Example: 10 episodes
        obs = env.reset()
        done = False
        total_reward = 0
        positions = []  # Store positions for the current episode
        while not done:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward
            pose = env.client.simGetVehiclePose().position
            positions.append((pose.x_val, pose.y_val, pose.z_val))  # Store the tuple of x, y, z positions
        reward_log.append(total_reward)
        all_positions.append(positions)
        print(f'Episode {i+1}: Total Reward: {total_reward}')

    # Save the model
    model.save("dqn_drone")

    # Evaluate and display results
    mean_reward, std_reward = evaluate_policy(model, model.get_env(), n_eval_episodes=10)
    print(f'Mean reward: {mean_reward}, Std reward: {std_reward}')

    # Plot the rewards and average trajectories
    plot_rewards(reward_log)
    plot_trajectories(all_positions)  # Plot trajectories for each episode

if __name__ == "__main__":
    main()
