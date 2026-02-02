#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import torch
from robot_env import RobotEnv
from models import DDPG

def main(args=None):
    rclpy.init(args=args)
    
    env = RobotEnv()
    
    agent = DDPG(
        env=env,
        lr_start=1e-4,
        lr_end=1e-5,
        lr_decay=0.0001,
        gamma=0.99,
        buffer_size=100000,
        batch_size=64,
        polyak_tau=0.005
    )

    num_episodes = 500
    max_steps_per_episode = 200

    try:
        for episode in range(num_episodes):
            state, _ = env.reset()
            episode_reward = 0
            
            for step in range(max_steps_per_episode):
                rclpy.spin_once(env, timeout_sec=0.01)
                
                action_tensor = agent.select_action(state)
                action = action_tensor.cpu().numpy()[0]
                
                next_state, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
                
                agent.buffer.push(state, action, reward, next_state, terminated)
                
                state = next_state
                episode_reward += reward
                
                if len(agent.buffer) > agent.batch_size:
                    agent.optimize()
                
                if done:
                    break
            
            print(f"Episode: {episode + 1} | Reward: {episode_reward:.2f} | Steps: {step + 1}")
            
            if (episode + 1) % 50 == 0:
                torch.save(agent.actor.state_dict(), f"actor_model_ep_{episode+1}.pth")

    except KeyboardInterrupt:
        pass
    finally:
        torch.save(agent.actor.state_dict(), "actor_model_final.pth")
        env.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()