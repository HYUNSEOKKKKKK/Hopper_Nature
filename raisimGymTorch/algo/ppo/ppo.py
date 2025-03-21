from datetime import datetime
import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.tensorboard import SummaryWriter
from .storage import RolloutStorage
from adamp import AdamP


class PPO:
    def __init__(self,
                 actor,
                 critic,
barrier_critic,
                 estimator,
                 num_envs,
                 num_transitions_per_env,
                 num_learning_epochs,
                 num_mini_batches,
                 clip_param=0.2,
                 gamma=0.998,
                 lam=0.95,
                 value_loss_coef=0.5,
                 entropy_coef=0.0,
                 learning_rate=5e-4,
                 max_grad_norm=0.5,
                 # learning_rate_schedule='adaptive',
                 desired_kl=0.01,
                 use_clipped_value_loss=True,
                 log_dir='run',
                 device='cpu',
                 shuffle_batch=True,
                 gradient_penalty_coef = 6e-4):

        # PPO components
        self.actor = actor
        self.critic = critic
        self.barrier_critic = barrier_critic
        self.estimator = estimator
        self.storage = RolloutStorage(num_envs, num_transitions_per_env, actor.obs_shape, critic.obs_shape, estimator.obs_shape,estimator.output_shape,actor.action_shape, device)

        if shuffle_batch:
            self.batch_sampler = self.storage.mini_batch_generator_shuffle
        else:
            self.batch_sampler = self.storage.mini_batch_generator_inorder

        # self.optimizer = optim.Adam([*self.actor.parameters(), *self.critic.parameters()], lr=learning_rate)
        self.optimizer = AdamP([*self.actor.parameters(), *self.critic.parameters(), *self.estimator.parameters(), *self.barrier_critic.parameters()], lr=learning_rate)
        self.device = device

        # env parameters
        self.num_transitions_per_env = num_transitions_per_env
        self.num_envs = num_envs

        # PPO parameters
        self.clip_param = clip_param
        self.num_learning_epochs = num_learning_epochs
        self.num_mini_batches = num_mini_batches
        self.value_loss_coef = value_loss_coef
        self.entropy_coef = entropy_coef
        self.gamma = gamma
        self.lam = lam
        self.max_grad_norm = max_grad_norm
        self.use_clipped_value_loss = use_clipped_value_loss

        # Gradient penalty loss
        self.gradient_penalty_coef = gradient_penalty_coef

        # Log
        self.log_dir = os.path.join(log_dir, datetime.now().strftime('%b%d_%H-%M-%S'))
        self.writer = SummaryWriter(log_dir=self.log_dir, flush_secs=10)
        self.tot_timesteps = 0
        self.tot_time = 0

        # ADAM
        # self.learning_rate = learning_rate
        # self.desired_kl = desired_kl
        # self.schedule = learning_rate_schedule

        # temps
        self.actions = None
        self.actions_log_prob = None
        self.actor_obs = None

        # Estimator
        self.mse_loss = nn.MSELoss()

    def act(self, actor_obs):
        self.actor_obs = actor_obs
        with torch.no_grad():
            self.actions, self.actions_log_prob = self.actor.sample(torch.from_numpy(actor_obs).to(self.device))
        return self.actions

    def step(self, value_obs, est_obs,true_state, rews, dones, bar_rews):
        self.storage.add_transitions(self.actor_obs, value_obs, est_obs,true_state, self.actions, rews, dones, bar_rews,
                                     self.actions_log_prob)

    def update(self, actor_obs, value_obs, log_this_iteration, update):
        last_values = self.critic.predict(torch.from_numpy(value_obs).to(self.device))
        last_barrier_values = self.barrier_critic.predict(torch.from_numpy(value_obs).to(self.device))
        # Learning step
        self.storage.compute_returns(last_values.to(self.device), self.critic, self.gamma, self.lam)
        self.storage.compute_barrier_returns(last_barrier_values.to(self.device), self.barrier_critic, self.gamma, self.lam)
        mean_value_loss, mean_surrogate_loss, mean_estimation_loss,mean_barrier_value_loss,mean_barrier_surrogate_loss, mean_GP_loss,infos = self._train_step(log_this_iteration)
        self.storage.clear()

        if log_this_iteration:
            self.log({**locals(), **infos, 'it': update})

    def _calc_grad_penalty(self, obs_batch, actions_log_prob_batch):
        grad_log_prob = torch.autograd.grad(actions_log_prob_batch.sum(), obs_batch, create_graph=True)[0]
        gradient_penalty_loss = torch.sum(torch.square(grad_log_prob), dim=-1).mean()
        return gradient_penalty_loss

    def log(self, variables):
        self.tot_timesteps += self.num_transitions_per_env * self.num_envs
        mean_std = self.actor.distribution.std.mean()
        self.writer.add_scalar('PPO/value_function', variables['mean_value_loss'], variables['it'])
        self.writer.add_scalar('PPO/barrier_value_function', variables['mean_barrier_value_loss'], variables['it'])
        self.writer.add_scalar('PPO/surrogate', variables['mean_surrogate_loss'], variables['it'])
        self.writer.add_scalar('PPO/barrier_surrogate', variables['mean_barrier_surrogate_loss'], variables['it'])
        self.writer.add_scalar('PPO/mean_noise_std', mean_std.item(), variables['it'])
        self.writer.add_scalar('PPO/estimation_loss', variables['mean_estimation_loss'], variables['it'])
        self.writer.add_scalar('PPO/gradient_penalty_loss', variables['mean_GP_loss'], variables['it'])
        # self.writer.add_scalar('PPO/learning_rate', self.learning_rate, variables['it'])

    def _train_step(self, log_this_iteration):
        mean_value_loss = 0
        mean_barrier_value_loss = 0
        mean_surrogate_loss = 0
        mean_barrier_surrogate_loss = 0
        mean_estimation_loss = 0
        mean_GP_loss = 0
        for epoch in range(self.num_learning_epochs):
            for (actor_obs_batch, critic_obs_batch, est_obs_batch, true_state_batch,actions_batch, current_values_batch, advantages_batch, returns_batch, old_actions_log_prob_batch,
                 current_barrier_values_batch, barrier_advantages_batch, barrier_returns_batch)\
                    in self.batch_sampler(self.num_mini_batches):
                actor_obs_batch.requires_grad_() # for gradient penalty loss computation

                actions_log_prob_batch, entropy_batch = self.actor.evaluate(actor_obs_batch, actions_batch)
                value_batch = self.critic.evaluate(critic_obs_batch)
                est_out_batch = self.estimator.evaluate(est_obs_batch)
                barrier_value_batch = self.barrier_critic.evaluate(critic_obs_batch)

                # Gradient penalty loss (regularization for d(output of action)/d(input of action))
                gradient_penalty_loss = self._calc_grad_penalty(actor_obs_batch, actions_log_prob_batch)

            # Estimator supervised loss
                estimation_loss = self.mse_loss(est_out_batch,true_state_batch)

                # Surrogate loss
                ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
                surrogate = -torch.squeeze(advantages_batch) * ratio
                surrogate_clipped = -torch.squeeze(advantages_batch) * torch.clamp(ratio, 1.0 - self.clip_param,
                                                                                   1.0 + self.clip_param)
                surrogate_loss = torch.max(surrogate, surrogate_clipped).mean()

                # Value function loss
                if self.use_clipped_value_loss:
                    value_clipped = current_values_batch + (value_batch - current_values_batch).clamp(-self.clip_param,
                                                                                                    self.clip_param)
                    value_losses = (value_batch - returns_batch).pow(2)
                    value_losses_clipped = (value_clipped - returns_batch).pow(2)
                    value_loss = torch.max(value_losses, value_losses_clipped).mean()
                else:
                    value_loss = (returns_batch - value_batch).pow(2).mean()

                # Barrier surrogate loss
                ratio = torch.exp(actions_log_prob_batch - torch.squeeze(old_actions_log_prob_batch))
                barrier_surrogate = -torch.squeeze(barrier_advantages_batch) * ratio
                barrier_surrogate_clipped = -torch.squeeze(barrier_advantages_batch) * torch.clamp(ratio, 1.0 - self.clip_param,
                                                                                   1.0 + self.clip_param)
                barrier_surrogate_loss = torch.max(barrier_surrogate, barrier_surrogate_clipped).mean()

                # Barrier value function loss
                if self.use_clipped_value_loss:
                    barrier_value_clipped = current_barrier_values_batch + (barrier_value_batch - current_barrier_values_batch).clamp(-self.clip_param,
                                                                                                      self.clip_param)
                    barrier_value_losses = (barrier_value_batch - barrier_returns_batch).pow(2)
                    barrier_value_losses_clipped = (barrier_value_clipped - barrier_returns_batch).pow(2)
                    barrier_value_loss = torch.max(barrier_value_losses, barrier_value_losses_clipped).mean()
                else:
                    barrier_value_loss = (barrier_returns_batch - barrier_value_batch).pow(2).mean()

                loss = (surrogate_loss + barrier_surrogate_loss+ self.value_loss_coef * value_loss - self.entropy_coef * entropy_batch.mean()+ estimation_loss * 0.1 + self.value_loss_coef * barrier_value_loss) + self.gradient_penalty_coef * gradient_penalty_loss

                # Gradient step
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_([*self.actor.parameters(), *self.critic.parameters(), *self.estimator.parameters(), *self.barrier_critic.parameters()], self.max_grad_norm)
                self.optimizer.step()

                if log_this_iteration:
                    mean_value_loss += value_loss.item()
                    mean_barrier_value_loss += barrier_value_loss.item()
                    mean_surrogate_loss += surrogate_loss.item()
                    mean_estimation_loss += estimation_loss.item()
                    mean_barrier_surrogate_loss += barrier_surrogate_loss.item()
                    mean_GP_loss += gradient_penalty_loss.item()

        if log_this_iteration:
            num_updates = self.num_learning_epochs * self.num_mini_batches
            mean_value_loss /= num_updates
            mean_barrier_value_loss /= num_updates
            mean_surrogate_loss /= num_updates
            mean_estimation_loss /= num_updates
            mean_barrier_surrogate_loss /= num_updates
            mean_GP_loss /= num_updates

        return mean_value_loss, mean_surrogate_loss, mean_estimation_loss, mean_barrier_value_loss,mean_barrier_surrogate_loss,mean_GP_loss, locals()
