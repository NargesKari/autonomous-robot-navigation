#!/usr/bin/env python3
import os
import sys
import argparse
import numpy as np
import time
from rl_agent import PPOAgent
from gym_gazebo_env import GazeboEnv


def evaluate_agent(
    model_path,
    num_episodes=10,
    world_name='depot',
    sim_steps_per_env_step=10,
    render=True,
    max_steps_per_episode=1000,
    device='cpu'
):
    print("=" * 60)
    print("RL Agent Evaluation")
    print("=" * 60)
    print(f"Model: {model_path}")
    print(f"Episodes: {num_episodes}")
    print(f"Max steps per episode: {max_steps_per_episode}")
    print(f"Device: {device}")
    print("=" * 60)

    if not os.path.exists(model_path):
        print(f"Error: Model file not found at {model_path}")
        return

    print("Creating environment...")
    env = GazeboEnv(
        world_name=world_name,
        cmd_topic='/cmd_vel',
        odom_topic='/wheel_encoder/odom',
        sim_steps_per_env_step=sim_steps_per_env_step,
        spawn_name='robot',
        spawn_pose=(0.0, 0.0, 0.9)
    )

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]

    print("Loading model...")
    agent = PPOAgent(state_dim=state_dim, action_dim=action_dim, device=device)
    agent.load(model_path)
    print("Model loaded successfully!")

    episode_rewards = []
    episode_lengths = []

    for episode in range(num_episodes):
        print(f"\n--- Episode {episode + 1}/{num_episodes} ---")
        obs, info = env.reset()
        
        episode_reward = 0.0
        episode_length = 0
        done = False

        while not done and episode_length < max_steps_per_episode:
            action = agent.get_action(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            
            episode_reward += reward
            episode_length += 1
            done = terminated or truncated

            if render and episode_length % 10 == 0:
                print(f"Step {episode_length}: pos=({obs[0]:.2f}, {obs[1]:.2f}), "
                      f"yaw={obs[2]:.2f}, reward={reward:.3f}, total={episode_reward:.3f}")

        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        
        print(f"Episode {episode + 1} completed:")
        print(f"  Total reward: {episode_reward:.3f}")
        print(f"  Episode length: {episode_length} steps")
        print(f"  Final position: ({obs[0]:.2f}, {obs[1]:.2f})")

    env.close()

    print("\n" + "=" * 60)
    print("Evaluation Summary")
    print("=" * 60)
    print(f"Mean reward: {np.mean(episode_rewards):.3f} ± {np.std(episode_rewards):.3f}")
    print(f"Mean episode length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    print(f"Best reward: {np.max(episode_rewards):.3f}")
    print(f"Worst reward: {np.min(episode_rewards):.3f}")
    print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluate trained RL agent')
    parser.add_argument('--model_path', type=str, required=True,
                        help='Path to trained model')
    parser.add_argument('--num_episodes', type=int, default=10,
                        help='Number of episodes to evaluate')
    parser.add_argument('--world_name', type=str, default='depot',
                        help='Gazebo world name')
    parser.add_argument('--sim_steps', type=int, default=10,
                        help='Simulator steps per environment step')
    parser.add_argument('--no_render', action='store_true',
                        help='Disable rendering/printing')
    parser.add_argument('--max_steps', type=int, default=1000,
                        help='Maximum steps per episode')
    parser.add_argument('--device', type=str, default='cpu',
                        help='Device to use (cpu or cuda)')
    
    args = parser.parse_args()

    evaluate_agent(
        model_path=args.model_path,
        num_episodes=args.num_episodes,
        world_name=args.world_name,
        sim_steps_per_env_step=args.sim_steps,
        render=not args.no_render,
        max_steps_per_episode=args.max_steps,
        device=args.device
    )
