#!/usr/bin/env python3
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

import numpy as np



class Actor(nn.Module):
    def __init__(self, state_dim, num_actions):
        super(Actor, self).__init__()

        self.negative_slope = 0.01

        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 128)
        self.fc4 = nn.Linear(128, 128)
        self.fc_action = nn.Linear(128, num_actions)

        # Initialize weights using He Initialization
        self._initialize_weights()
    
    def forward(self, x):
        x = F.leaky_relu(self.fc1(x), negative_slope=self.negative_slope)
        x = F.leaky_relu(self.fc2(x), negative_slope=self.negative_slope) + x
        x = F.leaky_relu(self.fc3(x), negative_slope=self.negative_slope)
        x = F.leaky_relu(self.fc4(x), negative_slope=self.negative_slope) + x
        actions = self.fc_action(x)
        actions = torch.tanh(actions)
        return actions

    def _initialize_weights(self):
        for name, layer in self.named_modules():
            if isinstance(layer, nn.Linear):
                if 'fc_action' in name:
                    nn.init.uniform_(layer.weight, -1e-3, 1e-3)
                    # nn.init.xavier_uniform_(layer.weight, gain=nn.init.calculate_gain('tanh'))
                    # nn.init.xavier_uniform_(layer.weight, gain=nn.init.calculate_gain('linear'))
                    if layer.bias is not None:
                        nn.init.zeros_(layer.bias)
                else:
                    # nn.init.uniform_(layer.weight, -1e-3, 1e-3)
                    nn.init.kaiming_normal_(layer.weight, a=self.negative_slope, nonlinearity="leaky_relu")
                    if layer.bias is not None:
                        nn.init.zeros_(layer.bias)




# Define the Dueling Q-Network
class QNetwork(nn.Module):
    def __init__(self, state_dim, num_actions):
        """
        Args:
            input_shape (int): Dimension of the input state.
            num_actions (int): Number of actions (output dimension).
        """
        super(QNetwork, self).__init__()

        self.negative_slope = 0.01

        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256 + num_actions, 256)
        self.fc3 = nn.Linear(256, 128)
        self.fc4 = nn.Linear(128, 128)
        self.fc_values = nn.Linear(128, 1)

        # Initialize weights using He Initialization
        self._initialize_weights()

    def forward(self, x, u):
        """
        Forward pass of the dueling network.

        Args:
            x (Tensor): Input tensor (state representation).

        Returns:
            Tensor: Q-values for each action.
        """

        # Value stream
        x = F.leaky_relu(self.fc1(x), negative_slope=self.negative_slope)
        x = torch.cat((x, u), dim=1)
        x = F.leaky_relu(self.fc2(x), negative_slope=self.negative_slope)
        x = F.leaky_relu(self.fc3(x), negative_slope=self.negative_slope)
        x = F.leaky_relu(self.fc4(x), negative_slope=self.negative_slope) + x
        q_values = self.fc_values(x)  # Output the state-value V(s)

        return q_values
    
    def _initialize_weights(self):
        for name, layer in self.named_modules():
            if isinstance(layer, nn.Linear):
                if 'fc_action' in name:
                    nn.init.uniform_(layer.weight, -1e-3, 1e-3)
                    # nn.init.xavier_uniform_(layer.weight, gain=nn.init.calculate_gain('tanh'))
                    # nn.init.xavier_uniform_(layer.weight, gain=nn.init.calculate_gain('linear'))
                    if layer.bias is not None:
                        nn.init.zeros_(layer.bias)
                else:
                    # nn.init.uniform_(layer.weight, -1e-3, 1e-3)
                    nn.init.kaiming_normal_(layer.weight, a=self.negative_slope, nonlinearity="leaky_relu")
                    if layer.bias is not None:
                        nn.init.zeros_(layer.bias)









class ReplayBuffer:
    def __init__(self, capacity, seed=None):
        self.capacity = capacity
        self.size = 0
        self.pos = 0  # Pointer to the next position to overwrite
        self.seed = seed
        self.rng = np.random.default_rng(seed=seed)

        # Pre-allocate memory for transitions
        self.states = np.zeros((capacity,), dtype=np.int64)
        self.actions = np.zeros((capacity,), dtype=np.int64)
        self.rewards = np.zeros((capacity,), dtype=np.float32)
        self.next_states = np.zeros((capacity,), dtype=np.int64)
        self.dones = np.zeros((capacity,), dtype=np.float32)

    def push(self, state, action, reward, next_state, done):
        """Store a transition in the buffer."""
        self.states[self.pos] = state
        self.actions[self.pos] = action
        self.rewards[self.pos] = reward
        self.next_states[self.pos] = next_state
        self.dones[self.pos] = done

        self.pos = (self.pos + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size):
        """Sample a random batch of transitions."""
        indices = self.rng.choice(self.size, size=batch_size, replace=False, shuffle=True)
        return (
            self.states[indices],
            self.actions[indices],
            self.rewards[indices],
            self.next_states[indices],
            self.dones[indices]
        )

    def __len__(self):
        return self.size








# DQN Agent
class DDPG:
    def __init__(self, env, 
                lr_start=1e-3, lr_end=1e-5, lr_decay=0.001, decay="linear", 
                gamma=0.99,
                Huberbeta=1.0, buffer_size=10000, batch_size=64, polyak_tau=0.05,
                seed=None, verbose=False):
        self.env = env
        self.observation_shape = env.observation_shape
        self.num_actions = env.action_space.n
        self.num_states = env.observation_space.n

        # Validate the decay type
        if decay not in {"linear", "exponential", "reciprocal"}:
            raise ValueError("Invalid value for decay. Must be 'linear', 'exponential' or 'reciprocal'.")

        self.lr = lr_start / env.n_active_features
        self.lr_start = lr_start / env.n_active_features
        self.lr_end = lr_end / env.n_active_features
        self.lr_decay = lr_decay 
        self.decay = decay

        self.noise_std = 1.0

        self.Huberbeta = Huberbeta
        self.gamma = gamma
        self.buffer_size = buffer_size
        self.batch_size = batch_size
        self.polyak_tau = polyak_tau
        self.seed = seed
        self.verbose = verbose
        self._is_training = True

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Main and Target networks
        self.qvalue_net = QNetwork(self.observation_shape, self.num_actions).to(self.device)
        self.target_net = QNetwork(self.observation_shape, self.num_actions).to(self.device)
        self.target_net.load_state_dict(self.qvalue_net.state_dict())
        self.target_net.eval()

        # self.optimizer = optim.Adam(self.qvalue_net.parameters(), lr=self.lr_start)
        self.optimizer = optim.AdamW(self.qvalue_net.parameters(), lr=self.lr_start, amsgrad=True)
        self.scheduler = optim.lr_scheduler.LambdaLR(self.optimizer, lr_lambda=self.update_learning_rate)
        self.criterion = nn.SmoothL1Loss(beta=Huberbeta)

        # actor network
        self.actor= Actor(self.observation_shape, self.num_actions).to(self.device)
        self.target_actor= Actor(self.observation_shape, self.num_actions).to(self.device)
        self.target_actor.load_state_dict(self.actor.state_dict())
        self.target_actor.eval()

        # self.actor_optimizer = optim.Adam(self.qvalue_net.parameters(), lr=self.lr_start)
        self.actor_optimizer = optim.AdamW(self.actor.parameters(), lr=self.lr_start, amsgrad=True)
        self.actor_scheduler = optim.lr_scheduler.LambdaLR(self.actor_optimizer, lr_lambda=self.update_learning_rate)

        # Prioritized Replay Buffer
        self.buffer = ReplayBuffer(capacity=buffer_size,
                                   seed=self.seed)

        self.steps = 0

        self.training()

    @property
    def is_training(self):
      return self._is_training

    def training(self):
      self._is_training = True

    def evaluating(self, seed=None):
      self._is_training = False

    @torch.no_grad()
    def select_action(self, state):
        state = torch.tensor(state, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            self.actor.eval()
            action = self.actor(state).detach()
            action = action + torch.randn_like(action) * self.noise_std
        return action

    def optimize(self):
        if len(self.buffer) < self.batch_size:
            return

        self.qvalue_net.train()

        # Sample from replay buffer with PER
        states, actions, rewards, next_states, dones = self.buffer.sample(self.batch_size)

        # Convert to tensors
        states = torch.tensor(states, dtype=torch.float32, device=self.device)
        actions = torch.tensor(actions, dtype=torch.int64, device=self.device).unsqueeze(1)
        rewards = torch.tensor(rewards, dtype=torch.float32, device=self.device).unsqueeze(1)
        next_states = torch.tensor(next_states, dtype=torch.float32, device=self.device)
        dones = torch.tensor(dones, dtype=torch.float32, device=self.device).unsqueeze(1)

        # Compute current Q values
        q_values = self.qvalue_net(states, actions)

       # Compute Double Q-Learning target Q values
        with torch.no_grad():
            # Use policy network to select action with max Q-value
            next_actions = self.target_actor(next_states)
            next_q_values = self.target_net(next_states, next_actions)

            # Compute target Q-values
            target_q_values = rewards + (1 - dones) * self.gamma * next_q_values

        # Compute TD errors and loss
        loss = self.criterion(q_values, target_q_values)

        # Optimize the model
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        # Update learning rate
        self.scheduler.step()

        # Update target network using Polyak averaging
        self.update_target_network()

        actor_loss = - torch.mean(self.qvalue_net(states, self.actor(states)))

         # Optimize the model
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # Update learning rate
        self.actor_scheduler.step()

        # Update target network using Polyak averaging
        self.actor_update_target_network()

    @torch.no_grad()
    def update_target_network(self):
        for target_param, online_param in zip(self.target_net.parameters(), self.qvalue_net.parameters()):
            target_param.data.copy_(self.polyak_tau * online_param.data + (1.0 - self.polyak_tau) * target_param.data)

    @torch.no_grad()
    def actor_update_target_network(self):
        for target_param, online_param in zip(self.target_actor.parameters(), self.actor.parameters()):
            target_param.data.copy_(self.polyak_tau * online_param.data + (1.0 - self.polyak_tau) * target_param.data)

    def train(self, num_episodes):
        episode_rewards = []
        episode_steps = []
        self.steps = 0

        for episode in range(num_episodes):
            state, info = self.env.reset()
            episode_reward = 0
            steps_in_episode = 0

            while True:
                self.steps += 1
                steps_in_episode += 1

                # Select action
                action = self.select_action(state)

                # Step environment
                next_state, reward, terminated, truncated, info = self.env.step(action)

                done = terminated or truncated

                # Store in replay buffer
                self.buffer.push(state, action, reward, next_state, terminated)
                state = next_state
                episode_reward += reward

                # Optimize the model
                self.optimize()

                if done:
                    break

            episode_rewards.append(episode_reward)
            episode_steps.append(steps_in_episode)
            print(f"Episode {episode + 1}, Reward: {episode_reward}, Steps: {steps_in_episode}")

        return episode_rewards, episode_steps

    def update_learning_rate(self, step):
        """
        Updates the learning rate value using the specified decay type.

        Args:
            steps (int): Current step count.

        Behavior:
            - Linear Decay: Epsilon decreases linearly with each step.
            - Exponential Decay: Epsilon decreases exponentially with each step.
            - Reciprocal Decay: Epsilon decreases inversely with each step.
        """
        if self.decay == "linear":
            # Linearly decay epsilon, ensuring it does not drop below epsilon_end
            self.lr = max(
                self.lr_end,
                self.lr_start - self.lr_decay * step 
            )
        elif self.decay == "exponential":
            # Exponentially decay epsilon, ensuring it does not drop below epsilon_end
            self.lr = max(
                self.lr_end,
                self.lr_start * np.exp(-self.lr_decay * step)
            )
        elif self.decay == "reciprocal":
            # Inversely decay epsilon, ensuring it does not drop below epsilon_end
            self.lr = max(
                self.lr_end,
                self.lr_start / (1 + self.lr_decay * step)
            )
        return self.lr / self.lr_start