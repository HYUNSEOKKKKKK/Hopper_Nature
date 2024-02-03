import torch
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
import numpy as np


class RolloutStorage:
    def __init__(self, num_envs, num_transitions_per_env, actor_obs_shape, critic_obs_shape, est_obs_shape, est_output_shape,actions_shape, device):
        self.device = device

        # Core
        self.critic_obs = np.zeros([num_transitions_per_env, num_envs, *critic_obs_shape], dtype=np.float32)
        self.actor_obs = np.zeros([num_transitions_per_env, num_envs, *actor_obs_shape], dtype=np.float32)
        self.est_obs = np.zeros([num_transitions_per_env, num_envs, *est_obs_shape], dtype=np.float32)
        self.true_state = np.zeros([num_transitions_per_env, num_envs, *est_output_shape()], dtype=np.float32)
        self.rewards = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.barrier_rewards = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.actions = np.zeros([num_transitions_per_env, num_envs, *actions_shape], dtype=np.float32)
        self.dones = np.zeros([num_transitions_per_env, num_envs, 1], dtype=bool)

        # For PPO
        self.actions_log_prob = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.values = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.returns = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.advantages = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)

        # for barrier
        self.barrier_values = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.barrier_returns = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)
        self.barrier_advantages = np.zeros([num_transitions_per_env, num_envs, 1], dtype=np.float32)

        # For Estimator
        self.est_obs_tc = torch.from_numpy(self.est_obs).to(self.device)
        self.true_state_tc = torch.from_numpy(self.true_state).to(self.device)

        # torch variables
        self.critic_obs_tc = torch.from_numpy(self.critic_obs).to(self.device)
        self.actor_obs_tc = torch.from_numpy(self.actor_obs).to(self.device)
        self.actions_tc = torch.from_numpy(self.actions).to(self.device)
        self.actions_log_prob_tc = torch.from_numpy(self.actions_log_prob).to(self.device)
        self.values_tc = torch.from_numpy(self.values).to(self.device)
        self.returns_tc = torch.from_numpy(self.returns).to(self.device)
        self.advantages_tc = torch.from_numpy(self.advantages).to(self.device)
        self.barrier_values_tc = torch.from_numpy(self.barrier_values).to(self.device)
        self.barrier_returns_tc = torch.from_numpy(self.barrier_returns).to(self.device)
        self.barrier_advantages_tc = torch.from_numpy(self.barrier_advantages).to(self.device)

        self.num_transitions_per_env = num_transitions_per_env
        self.num_envs = num_envs
        self.device = device

        self.step = 0

    def add_transitions(self, actor_obs, critic_obs, est_obs,true_state, actions, rewards, dones, barrier_rewards,actions_log_prob):
        if self.step >= self.num_transitions_per_env:
            raise AssertionError("Rollout buffer overflow")
        self.critic_obs[self.step] = critic_obs
        self.actor_obs[self.step] = actor_obs
        self.est_obs[self.step] = est_obs
        self.true_state[self.step] = true_state
        self.actions[self.step] = actions
        self.rewards[self.step] = rewards.reshape(-1, 1)
        self.dones[self.step] = dones.reshape(-1, 1)
        self.barrier_rewards[self.step] = barrier_rewards.reshape(-1, 1)
        self.actions_log_prob[self.step] = actions_log_prob.reshape(-1, 1)
        self.step += 1

    def clear(self):
        self.step = 0

    def compute_returns(self, last_values, critic, gamma, lam):
        with torch.no_grad():
            self.values = critic.predict(torch.from_numpy(self.critic_obs).to(self.device)).cpu().numpy()

        advantage = 0

        for step in reversed(range(self.num_transitions_per_env)):
            if step == self.num_transitions_per_env - 1:
                next_values = last_values.cpu().numpy()
                # next_is_not_terminal = 1.0 - self.dones[step].float()
            else:
                next_values = self.values[step + 1]
                # next_is_not_terminal = 1.0 - self.dones[step+1].float()

            next_is_not_terminal = 1.0 - self.dones[step]
            delta = self.rewards[step] + next_is_not_terminal * gamma * next_values - self.values[step]
            advantage = delta + next_is_not_terminal * gamma * lam * advantage
            self.returns[step] = advantage + self.values[step]

        # Compute and normalize the advantages
        self.advantages = self.returns - self.values
        self.advantages = (self.advantages - self.advantages.mean()) / (self.advantages.std() + 1e-8)
        # Convert to torch variables
        self.critic_obs_tc = torch.from_numpy(self.critic_obs).to(self.device)
        self.actor_obs_tc = torch.from_numpy(self.actor_obs).to(self.device)
        self.est_obs_tc = torch.from_numpy(self.est_obs).to(self.device)
        self.true_state_tc = torch.from_numpy(self.true_state).to(self.device)
        self.actions_tc = torch.from_numpy(self.actions).to(self.device)
        self.actions_log_prob_tc = torch.from_numpy(self.actions_log_prob).to(self.device)
        self.values_tc = torch.from_numpy(self.values).to(self.device)
        self.returns_tc = torch.from_numpy(self.returns).to(self.device)
        self.advantages_tc = torch.from_numpy(self.advantages).to(self.device)
    def compute_barrier_returns(self, last_values, barrier_critic, gamma, lam):
        with torch.no_grad():
            self.barrier_values = barrier_critic.predict(torch.from_numpy(self.critic_obs).to(self.device)).cpu().numpy()

        advantage = 0

        for step in reversed(range(self.num_transitions_per_env)):
            if step == self.num_transitions_per_env - 1:
                next_values = last_values.cpu().numpy()
                # next_is_not_terminal = 1.0 - self.dones[step].float()
            else:
                next_values = self.barrier_values[step + 1]
                # next_is_not_terminal = 1.0 - self.dones[step+1].float()

            next_is_not_terminal = 1.0 - self.dones[step]
            # next_is_not_terminal = 1.0
            delta = self.barrier_rewards[step] + next_is_not_terminal * gamma * next_values - self.barrier_values[step]
            advantage = delta + next_is_not_terminal * gamma * lam * advantage
            self.barrier_returns[step] = advantage + self.barrier_values[step]

        # Compute and normalize the advantages
        self.barrier_advantages = self.barrier_returns - self.barrier_values
        self.barrier_advantages = (self.barrier_advantages - self.barrier_advantages.mean()) / (self.barrier_advantages.std() + 1e-8)
        # Convert to torch variables
        self.barrier_values_tc = torch.from_numpy(self.barrier_values).to(self.device)
        self.barrier_returns_tc = torch.from_numpy(self.barrier_returns).to(self.device)
        self.barrier_advantages_tc = torch.from_numpy(self.barrier_advantages).to(self.device)

    def mini_batch_generator_shuffle(self, num_mini_batches):
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches

        for indices in BatchSampler(SubsetRandomSampler(range(batch_size)), mini_batch_size, drop_last=True):
            actor_obs_batch = self.actor_obs_tc.view(-1, *self.actor_obs_tc.size()[2:])[indices]
            critic_obs_batch = self.critic_obs_tc.view(-1, *self.critic_obs_tc.size()[2:])[indices]
            est_obs_batch = self.est_obs_tc.view(-1, *self.est_obs_tc.size()[2:])[indices]
            true_state_batch = self.true_state_tc.view(-1, *self.true_state_tc.size()[2:])[indices]
            actions_batch = self.actions_tc.view(-1, self.actions_tc.size(-1))[indices]
            values_batch = self.values_tc.view(-1, 1)[indices]
            returns_batch = self.returns_tc.view(-1, 1)[indices]
            old_actions_log_prob_batch = self.actions_log_prob_tc.view(-1, 1)[indices]
            advantages_batch = self.advantages_tc.view(-1, 1)[indices]

            barrier_values_batch = self.barrier_values_tc.view(-1, 1)[indices]
            barrier_returns_batch = self.barrier_returns_tc.view(-1, 1)[indices]
            barrier_advantages_batch = self.barrier_advantages_tc.view(-1, 1)[indices]
            yield actor_obs_batch, critic_obs_batch, est_obs_batch, true_state_batch, actions_batch, values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch,barrier_values_batch,barrier_advantages_batch,barrier_returns_batch

    def mini_batch_generator_inorder(self, num_mini_batches):
        batch_size = self.num_envs * self.num_transitions_per_env
        mini_batch_size = batch_size // num_mini_batches

        for batch_id in range(num_mini_batches):
            yield self.actor_obs_tc.view(-1, *self.actor_obs_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.critic_obs_tc.view(-1, *self.critic_obs_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.est_obs_tc.view(-1, *self.est_obs_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.true_state_tc.view(-1, *self.true_state_tc.size()[2:])[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.actions_tc.view(-1, self.actions_tc.size(-1))[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.values_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.advantages_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.returns_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.actions_log_prob_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.barrier_values_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.barrier_advantages_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size], \
                self.barrier_returns_tc.view(-1, 1)[batch_id*mini_batch_size:(batch_id+1)*mini_batch_size]
