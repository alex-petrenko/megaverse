import cv2
import gym
import numpy as np

from gym.spaces import Discrete

# noinspection PyUnresolvedReferences
from voxel_env.extension.voxel_env import VoxelEnvGym, set_voxel_env_log_level


class VoxelEnv(gym.Env):
    def __init__(self, num_envs, num_agents_per_env, num_simulation_threads, vertical_look_limit_rad=0.0, use_vulkan=False):
        self.is_multiagent = True

        set_voxel_env_log_level(2)

        self.img_w = 128
        self.img_h = 72
        self.channels = 3

        self.use_vulkan = use_vulkan

        # total number of simulated agents
        self.num_agents = num_envs * num_agents_per_env
        self.num_envs = num_envs
        self.num_agents_per_env = num_agents_per_env

        self.env = VoxelEnvGym(
            self.img_w, self.img_h,
            num_envs, num_agents_per_env, num_simulation_threads,
            vertical_look_limit_rad, use_vulkan,
        )

        # obtaining default reward shaping scheme
        self.default_shaping_scheme = self.env.get_reward_shaping(0, 0)

        self.action_space = self.generate_action_space(self.env.action_space_sizes())
        self.observation_space = gym.spaces.Box(0, 255, (self.channels, self.img_h, self.img_w), dtype=np.uint8)

    @staticmethod
    def generate_action_space(action_space_sizes):
        """
        Left = 1 << 1,
        Right = 1 << 2,

        Forward = 1 << 3,
        Backward = 1 << 4,

        LookLeft = 1 << 5,
        LookRight = 1 << 6,

        Jump = 1 << 7,
        Interact = 1 << 8,

        LookDown = 1 << 9,
        LookUp = 1 << 10,
        """
        # spaces = [
        #     Discrete(3),  # noop, go left, go right
        #     Discrete(3),  # noop, forward, backward
        #     Discrete(3),  # noop, look left, look right
        #     Discrete(2),  # noop, jump
        #     Discrete(2),  # noop, interact
        #     Discrete(3),  # noop, look down, look up
        # ]

        spaces = [Discrete(sz) for sz in action_space_sizes]
        space = gym.spaces.Tuple(spaces)
        return space

    def seed(self, seed=None):
        if seed is None:
            return

        assert isinstance(seed, int), 'Expect seed to be an integer'
        self.env.seed(seed)

    def observations(self):
        obs = []
        for env_i in range(self.num_envs):
            for agent_i in range(self.num_agents_per_env):
                o = self.env.get_observation(env_i, agent_i)
                o = o[:, :, :3]
                o = np.transpose(o, (2, 0, 1))  # convert to CHW for PyTorch
                obs.append(o)

        return obs

    def reset(self):
        self.env.reset()
        return self.observations()

    # def set_agent_actions(self, env_i, agent_i, actions):
    #     action_idx = 0
    #     action_mask = 0
    #     spaces = self.action_space.spaces
    #     for i, action in enumerate(actions):
    #         if action > 0:
    #             action_mask = action_mask | (1 << (action_idx + action))
    #         num_non_idle_actions = spaces[i].n - 1
    #         action_idx += num_non_idle_actions
    #
    #     self.env.set_action_mask(env_i, agent_i, action_mask)

    def step(self, actions):
        action_idx = 0
        for env_i in range(self.num_envs):
            for agent_i in range(self.num_agents_per_env):
                # self.set_agent_actions(env_i, agent_i, actions[action_idx])
                self.env.set_actions(env_i, agent_i, actions[action_idx])
                action_idx += 1

        self.env.step()

        dones, infos = [], []

        agent_i = 0
        for env_i in range(self.num_envs):
            done = self.env.is_done(env_i)  # currently no individual done per agent
            dones.extend([done for _ in range(self.num_agents_per_env)])
            if done:
                true_objective = self.env.true_objective(env_i)
                infos.extend([dict(true_reward=float(true_objective)) for _ in range(self.num_agents_per_env)])
            else:
                infos.extend([{} for _ in range(self.num_agents_per_env)])

            agent_i += self.num_agents_per_env

        rewards = self.env.get_last_rewards()

        obs = self.observations()

        return obs, rewards, dones, infos

    def convert_obs(self, obs):
        if not self.use_vulkan:
            obs = cv2.flip(obs, 0)
        obs = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)
        return obs

    def render(self, mode='human'):
        self.env.draw_hires()

        rows = []
        for env_i in range(self.num_envs):
            obs = [self.convert_obs(self.env.get_hires_observation(env_i, i)) for i in range(self.num_agents_per_env)]
            obs_concat = np.concatenate(obs, axis=1)
            rows.append(obs_concat)

        obs_final = np.concatenate(rows, axis=0)
        cv2.imshow(f'agent_{id(self)}', obs_final)
        cv2.waitKey(1)

    def get_default_reward_shaping(self):
        return self.default_shaping_scheme

    def get_current_reward_shaping(self, actor_idx: int):
        env_idx = actor_idx // self.num_agents_per_env
        agent_idx = actor_idx % self.num_agents_per_env
        return self.env.get_reward_shaping(env_idx, agent_idx)

    def set_reward_shaping(self, reward_shaping: dict, actor_idx: int):
        env_idx = actor_idx // self.num_agents_per_env
        agent_idx = actor_idx % self.num_agents_per_env
        return self.env.set_reward_shaping(env_idx, agent_idx, reward_shaping)

    def close(self):
        if self.env:
            self.env.close()
