import cv2
import gym
import numpy as np

from gym.spaces import Discrete

# noinspection PyUnresolvedReferences
from voxel_env.extension.voxel_env import VoxelEnvGym, set_voxel_env_log_level


class VoxelEnv(gym.Env):
    def __init__(self, num_agents=2, vertical_look_limit_rad=0.0, use_vulkan=False):
        set_voxel_env_log_level(2)

        self.img_w = 128
        self.img_h = 72
        self.channels = 3

        self.use_vulkan = use_vulkan

        self.num_agents = num_agents
        self.env = VoxelEnvGym(self.img_w, self.img_h, self.num_agents, vertical_look_limit_rad, use_vulkan)

        self.empty_infos = [{} for _ in range(self.num_agents)]

        self.action_space = self.generate_action_space()
        self.observation_space = gym.spaces.Box(0, 255, (self.channels, self.img_h, self.img_w), dtype=np.uint8)

    @staticmethod
    def generate_action_space():
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
        spaces = [
            Discrete(3),  # noop, go left, go right
            Discrete(3),  # noop, forward, backward
            Discrete(3),  # noop, look left, look right
            Discrete(2),  # noop, jump
            Discrete(2),  # noop, interact
            Discrete(3),  # noop, look down, look up
        ]

        space = gym.spaces.Tuple(spaces)
        return space

    def seed(self, seed=None):
        if seed is None:
            return

        assert isinstance(seed, int), 'Expect seed to be an integer'
        self.env.seed(seed)

    def observations(self):
        obs = []
        for i in range(self.num_agents):
            o = self.env.get_observation(i)
            o = o[:, :, :3]
            o = np.transpose(o, (2, 0, 1))  # convert to CHW for PyTorch
            obs.append(o)

        return obs

    def reset(self):
        self.env.reset()
        return self.observations()

    def set_agent_actions(self, agent_idx, actions):
        action_idx = 0
        action_mask = 0
        spaces = self.action_space.spaces
        for i, action in enumerate(actions):
            if action > 0:
                action_mask = action_mask | (1 << (action_idx + action))
            num_non_idle_actions = spaces[i].n - 1
            action_idx += num_non_idle_actions

        self.env.set_action_mask(agent_idx, action_mask)

    def step(self, actions):
        for i, agent_actions in enumerate(actions):
            self.set_agent_actions(i, agent_actions)

        done = self.env.step()
        dones = [done for _ in range(self.num_agents)]
        rewards = [self.env.get_last_reward(i) for i in range(self.num_agents)]

        if done:
            true_objective = self.env.true_objective()
            infos = [dict(true_reward=float(true_objective)) for _ in range(self.num_agents)]
            obs = self.reset()
        else:
            obs = self.observations()
            infos = self.empty_infos

        return obs, rewards, dones, infos

    def convert_obs(self, obs):
        if not self.use_vulkan:
            obs = cv2.flip(obs, 0)
        obs = cv2.cvtColor(obs, cv2.COLOR_RGB2BGR)
        return obs

    def render(self, mode='human'):
        self.env.draw_hires()
        max_num_rows = 2
        num_rows = min(max_num_rows, self.num_agents // 2)
        num_cols = self.num_agents // num_rows

        rows = []
        for row in range(num_rows):
            obs = [self.convert_obs(self.env.get_hires_observation(i + row * num_cols)) for i in range(num_cols)]
            obs_concat = np.concatenate(obs, axis=1)
            rows.append(obs_concat)

        obs_final = np.concatenate(rows, axis=0)
        cv2.imshow(f'agent_{0}_{id(self)}', obs_final)
        cv2.waitKey(1)

    def close(self):
        if self.env:
            self.env.close()
