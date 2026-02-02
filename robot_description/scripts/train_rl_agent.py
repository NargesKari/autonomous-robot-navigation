#!/usr/bin/env python3
import os
import sys
import argparse
import numpy as np
import torch
from datetime import datetime
from collections import deque
from rl_agent import PPOAgent
from gym_gazebo_env import GazeboEnv


def train_agent(
    total_timesteps=100000,
    learning_rate=3e-4,
    batch_size=64,
    n_epochs=10,
    gamma=0.99,
    gae_lambda=0.95,
    clip_epsilon=0.2,
    value_coef=0.5,
    entropy_coef=0.01,
    max_grad_norm=0.5,
    world_name='depot',
    sim_steps_per_env_step=10,
    model_save_path='./models/rl_path_follower',
    log_interval=1000,
    save_interval=10000,
    continue_training=False,
    load_model_path=None,
    device='cpu'
):
    start_time = datetime.now()
    print("=" * 60)
    print("RL Agent Training Setup")
    print("=" * 60)
    print(f"Start time: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Total timesteps: {total_timesteps}")
    print(f"Learning rate: {learning_rate}")
    print(f"World: {world_name}")
    print(f"Sim steps per env step: {sim_steps_per_env_step}")
    print(f"Device: {device}")
    print("=" * 60)

    os.makedirs(model_save_path, exist_ok=True)

    print("\n" + "="*60)
    print("ENVIRONMENT SETUP")
    print("="*60)
    print("Creating environment...")
    print(f"  World: {world_name}")
    print(f"  Command topic: /cmd_vel")
    print(f"  Odometry topic: /wheel_encoder/odom")
    print(f"  ControlWorld service: /world/{world_name}/control")
    print("\nChecking prerequisites...")
    print("  [ ] ROS 2 environment sourced")
    print("  [ ] Gazebo simulation running")
    print("  [ ] ControlWorld service bridge active")
    print("  [ ] Odometry topic publishing")
    print("="*60 + "\n")
    
    try:
        env = GazeboEnv(
            world_name=world_name,
            cmd_topic='/cmd_vel',
            odom_topic='/wheel_encoder/odom',
            sim_steps_per_env_step=sim_steps_per_env_step,
            spawn_name='robot',
            spawn_pose=(0.0, 0.0, 0.9)
        )
        print("✓ Environment created successfully!")
    except RuntimeError as e:
        print(f"\n❌ ERROR: Environment creation failed!")
        print(f"   {str(e)}")
        print("\nTroubleshooting:")
        print("  1. Make sure Gazebo is running:")
        print("     ros2 launch robot_description rl_control.launch.py")
        print("  2. Check if ControlWorld service is available:")
        print("     ros2 service list | grep control")
        print("  3. Verify odometry is publishing:")
        print("     ros2 topic echo /wheel_encoder/odom")
        print("  4. Check ROS 2 is sourced:")
        print("     echo $ROS_DISTRO")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ ERROR: Unexpected error during environment creation!")
        print(f"   {type(e).__name__}: {str(e)}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    print(f"State dimension: {state_dim}")
    print(f"Action dimension: {action_dim}")

    print("Creating PPO agent...")
    agent = PPOAgent(
        state_dim=state_dim,
        action_dim=action_dim,
        lr=learning_rate,
        gamma=gamma,
        gae_lambda=gae_lambda,
        clip_epsilon=clip_epsilon,
        value_coef=value_coef,
        entropy_coef=entropy_coef,
        max_grad_norm=max_grad_norm,
        device=device
    )

    if continue_training and load_model_path and os.path.exists(load_model_path):
        print(f"Loading existing model from {load_model_path}")
        agent.load(load_model_path)

    print("\nStarting training...")
    print("Press Ctrl+C to stop training early (model will be saved)")
    print(f"Progress will be logged every {log_interval} timesteps and at episode completion\n")
    
    episode_rewards = []
    episode_lengths = []
    episode_reward = 0
    episode_length = 0
    
    states = []
    actions = []
    old_log_probs = []
    rewards = []
    dones = []
    values = []
    
    timestep = 0
    episode = 0
    
    try:
        print("Resetting environment for first episode...")
        obs, info = env.reset()
        print(f"✓ Environment reset successful!")
        print(f"  Starting position: ({obs[0]:.2f}, {obs[1]:.2f})")
        print(f"  Starting yaw: {obs[2]:.2f} rad ({np.degrees(obs[2]):.1f}°)")
        print(f"  Observation shape: {obs.shape}")
        print("\n" + "="*60)
        print("TRAINING CONFIGURATION")
        print("="*60)
        print("Current Reward Function: Negative distance to origin")
        print("  Reward = -sqrt(x² + y²)")
        print("  Goal: Learn to navigate toward (0, 0)")
        print("\nNOTE: This is NOT path following - it's basic navigation.")
        print("      The agent learns to move toward the origin.")
        print("      For path following, the reward function needs to be updated.")
        print("="*60 + "\n")
        
        while timestep < total_timesteps:
            state = obs.copy()
            
            action = agent.get_action(state, deterministic=False)
            
            # Defensive clamp to ensure within bounds
            action = np.clip(action, [-1.0, -3.14], [1.0, 3.14])
            
            action_mean, value = agent.policy(torch.FloatTensor(state).unsqueeze(0).to(device))
            action_std = torch.exp(agent.policy.log_std) * agent.policy.action_scale
            dist = torch.distributions.Normal(action_mean, action_std)
            action_tensor = torch.FloatTensor(action).unsqueeze(0).to(device)
            log_prob = dist.log_prob(action_tensor).sum(axis=-1).item()
            value = value.item()
            
            next_obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            states.append(state)
            actions.append(action)
            old_log_probs.append(log_prob)
            rewards.append(reward)
            dones.append(done)
            values.append(value)
            
            episode_reward += reward
            episode_length += 1
            timestep += 1
            obs = next_obs
            
            if done or len(states) >= batch_size:
                if done:
                    next_value = 0.0
                else:
                    _, next_value = agent.policy(torch.FloatTensor(obs).unsqueeze(0).to(device))
                    next_value = next_value.item()
                
                if len(states) > 0:
                    for epoch in range(n_epochs):
                        stats = agent.update(states, actions, old_log_probs, rewards, dones, values, next_value)
                    
                    if timestep % log_interval == 0 or done:
                        progress_pct = (timestep / total_timesteps) * 100
                        avg_reward = np.mean(episode_rewards[-10:]) if len(episode_rewards) >= 10 else (np.mean(episode_rewards) if episode_rewards else 0)
                        avg_length = np.mean(episode_lengths[-10:]) if len(episode_lengths) >= 10 else (np.mean(episode_lengths) if episode_lengths else 0)
                        
                        elapsed_time = datetime.now() - start_time
                        if timestep > 0:
                            time_per_step = elapsed_time.total_seconds() / timestep
                            remaining_steps = total_timesteps - timestep
                            eta_seconds = time_per_step * remaining_steps
                            eta_str = f"{int(eta_seconds//3600)}h {int((eta_seconds%3600)//60)}m {int(eta_seconds%60)}s"
                        else:
                            eta_str = "calculating..."
                        
                        print(f"\n{'='*70}")
                        print(f"Training Progress - {progress_pct:.1f}% ({timestep}/{total_timesteps} timesteps)")
                        print(f"{'='*70}")
                        print(f"Episode: {episode} | Current Reward: {episode_reward:.2f} | Avg Reward (last 10): {avg_reward:.2f}")
                        print(f"Episode Length: {episode_length} | Avg Length (last 10): {avg_length:.1f}")
                        print(f"Loss - Total: {stats['total_loss']:.4f} | Actor: {stats['actor_loss']:.4f} | Critic: {stats['critic_loss']:.4f} | Entropy: {stats['entropy']:.4f}")
                        print(f"Elapsed: {str(elapsed_time).split('.')[0]} | ETA: {eta_str}")
                        print(f"{'='*70}\n")
                
                if done:
                    episode_rewards.append(episode_reward)
                    episode_lengths.append(episode_length)
                    episode += 1
                    episode_reward = 0
                    episode_length = 0
                    obs, info = env.reset()
                
                states = []
                actions = []
                old_log_probs = []
                rewards = []
                dones = []
                values = []
            
            if timestep % save_interval == 0:
                checkpoint_path = os.path.join(model_save_path, f"rl_model_{timestep}_steps.pth")
                agent.save(checkpoint_path)
                print(f"Model saved to {checkpoint_path}")
    
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("TRAINING INTERRUPTED BY USER")
        print("=" * 60)
        interrupted = True
    else:
        interrupted = False

    end_time = datetime.now()
    training_duration = end_time - start_time

    print("\n" + "=" * 60)
    print("TRAINING COMPLETED")
    print("=" * 60)
    print(f"End time: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"Training duration: {training_duration}")
    print(f"Total timesteps completed: {timestep}/{total_timesteps}")
    
    if interrupted:
        print("Status: INTERRUPTED (training stopped early)")
    else:
        print("Status: COMPLETED (all timesteps finished)")

    final_model_path = os.path.join(model_save_path, "rl_model_final.pth")
    agent.save(final_model_path)
    print(f"\nModel saved to: {final_model_path}")
    
    if episode_rewards:
        print(f"\n" + "-" * 60)
        print("TRAINING STATISTICS")
        print("-" * 60)
        print(f"Total episodes completed: {episode}")
        print(f"Mean episode reward: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
        print(f"Mean episode length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f} steps")
        print(f"Best episode reward: {np.max(episode_rewards):.2f}")
        print(f"Worst episode reward: {np.min(episode_rewards):.2f}")
        if len(episode_rewards) > 1:
            recent_rewards = episode_rewards[-10:] if len(episode_rewards) >= 10 else episode_rewards
            print(f"Recent (last {len(recent_rewards)}) mean reward: {np.mean(recent_rewards):.2f}")
        print("-" * 60)
    else:
        print("\nNo episodes completed - check environment setup")

    print("\n" + "=" * 60)
    print("Training finished! You can now evaluate the model with:")
    print(f"  python3 evaluate_rl_agent.py --model_path {final_model_path}")
    print("=" * 60)

    env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Train RL agent for path following')
    parser.add_argument('--total_timesteps', type=int, default=100000,
                        help='Total number of training timesteps')
    parser.add_argument('--learning_rate', type=float, default=3e-4,
                        help='Learning rate')
    parser.add_argument('--batch_size', type=int, default=64,
                        help='Batch size for updates')
    parser.add_argument('--n_epochs', type=int, default=10,
                        help='Number of epochs per update')
    parser.add_argument('--world_name', type=str, default='depot',
                        help='Gazebo world name')
    parser.add_argument('--sim_steps', type=int, default=10,
                        help='Simulator steps per environment step')
    parser.add_argument('--model_path', type=str, default='./models/rl_path_follower',
                        help='Path to save/load model')
    parser.add_argument('--log_interval', type=int, default=1000,
                        help='Logging interval')
    parser.add_argument('--save_interval', type=int, default=10000,
                        help='Model save interval')
    parser.add_argument('--continue_training', action='store_true',
                        help='Continue training from existing model')
    parser.add_argument('--load_model', type=str, default=None,
                        help='Path to model to continue training from')
    parser.add_argument('--device', type=str, default='cpu',
                        help='Device to use (cpu or cuda)')
    
    args = parser.parse_args()

    train_agent(
        total_timesteps=args.total_timesteps,
        learning_rate=args.learning_rate,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        world_name=args.world_name,
        sim_steps_per_env_step=args.sim_steps,
        model_save_path=args.model_path,
        log_interval=args.log_interval,
        save_interval=args.save_interval,
        continue_training=args.continue_training,
        load_model_path=args.load_model,
        device=args.device
    )
