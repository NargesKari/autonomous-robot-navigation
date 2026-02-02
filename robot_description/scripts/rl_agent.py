#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np
from collections import deque
import random


class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(ActorCritic, self).__init__()
        
        self.shared = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )
        
        self.actor = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, action_dim),
            nn.Tanh()
        )
        
        self.critic = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 1)
        )
        
        self.log_std = nn.Parameter(torch.zeros(action_dim))
        
        # Action scaling: [linear_scale, angular_scale] to match action space bounds
        # Linear: [-1.0, 1.0], Angular: [-3.14, 3.14]
        self.register_buffer('action_scale', torch.tensor([1.0, 3.14]))
        self.register_buffer('action_low', torch.tensor([-1.0, -3.14]))
        self.register_buffer('action_high', torch.tensor([1.0, 3.14]))
    
    def forward(self, state):
        shared_out = self.shared(state)
        action_mean_raw = self.actor(shared_out)
        # Scale Tanh output to action space bounds
        action_mean = action_mean_raw * self.action_scale
        value = self.critic(shared_out)
        return action_mean, value
    
    def get_action_and_value(self, state, action=None):
        action_mean, value = self.forward(state)
        # Scale std to match action space
        action_std = torch.exp(self.log_std) * self.action_scale
        
        dist = torch.distributions.Normal(action_mean, action_std)
        if action is None:
            action = dist.sample()
        
        # Clamp action to valid bounds
        action = torch.clamp(action, self.action_low, self.action_high)
        
        log_prob = dist.log_prob(action).sum(axis=-1)
        entropy = dist.entropy().sum(axis=-1)
        
        return action, log_prob, entropy, value


class PPOAgent:
    def __init__(
        self,
        state_dim,
        action_dim,
        lr=3e-4,
        gamma=0.99,
        gae_lambda=0.95,
        clip_epsilon=0.2,
        value_coef=0.5,
        entropy_coef=0.01,
        max_grad_norm=0.5,
        device='cpu'
    ):
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.value_coef = value_coef
        self.entropy_coef = entropy_coef
        self.max_grad_norm = max_grad_norm
        
        self.device = device
        self.policy = ActorCritic(state_dim, action_dim).to(device)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=lr)
        
    def get_action(self, state, deterministic=False):
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
        with torch.no_grad():
            action_mean, _ = self.policy(state_tensor)
            
            if deterministic:
                action = action_mean
            else:
                # Scale std to match action space
                action_std = torch.exp(self.policy.log_std) * self.policy.action_scale
                dist = torch.distributions.Normal(action_mean, action_std)
                action = dist.sample()
            
            # Clamp to action space bounds
            action = torch.clamp(action, 
                                self.policy.action_low.to(action.device),
                                self.policy.action_high.to(action.device))
            
            return action.cpu().numpy()[0]
    
    def compute_gae(self, rewards, values, next_value, dones):
        advantages = []
        gae = 0
        next_value = next_value
        
        for step in reversed(range(len(rewards))):
            delta = rewards[step] + self.gamma * next_value * (1 - dones[step]) - values[step]
            gae = delta + self.gamma * self.gae_lambda * (1 - dones[step]) * gae
            advantages.insert(0, gae)
            next_value = values[step]
        
        returns = [adv + val for adv, val in zip(advantages, values)]
        return advantages, returns
    
    def update(self, states, actions, old_log_probs, rewards, dones, values, next_value):
        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        old_log_probs = torch.FloatTensor(old_log_probs).to(self.device)
        
        advantages, returns = self.compute_gae(rewards, values, next_value, dones)
        advantages = torch.FloatTensor(advantages).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)
        
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        _, new_log_probs, entropy, new_values = self.policy.get_action_and_value(states, actions)
        
        ratio = torch.exp(new_log_probs - old_log_probs)
        surr1 = ratio * advantages
        surr2 = torch.clamp(ratio, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * advantages
        actor_loss = -torch.min(surr1, surr2).mean()
        
        critic_loss = F.mse_loss(new_values.squeeze(), returns)
        
        loss = actor_loss + self.value_coef * critic_loss - self.entropy_coef * entropy.mean()
        
        self.optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
        self.optimizer.step()
        
        return {
            'actor_loss': actor_loss.item(),
            'critic_loss': critic_loss.item(),
            'entropy': entropy.mean().item(),
            'total_loss': loss.item()
        }
    
    def save(self, path):
        torch.save({
            'policy_state_dict': self.policy.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
        }, path)
    
    def load(self, path):
        checkpoint = torch.load(path, map_location=self.device)
        self.policy.load_state_dict(checkpoint['policy_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
