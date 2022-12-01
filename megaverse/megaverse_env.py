import cv2
import gym
import numpy as np

from gym.spaces import Discrete

# noinspection PyUnresolvedReferences
from megaverse.extension.megaverse import MegaverseGym, set_megaverse_log_level


MEGAVERSE8 = [
    'TowerBuilding',
    'ObstaclesEasy',
    'ObstaclesHard',
    'Collect',
    'Sokoban',
    'HexMemory',
    'HexExplore',
    'Rearrange',
]

OBSTACLES_MULTITASK = [
    'ObstaclesWalls', 'ObstaclesSteps', 'ObstaclesLava', 'ObstaclesEasy', 'ObstaclesHard',
]


def make_env_multitask(multitask_name, task_idx, num_envs, num_agents_per_env, num_simulation_threads, use_vulkan=False, params=None):
    assert 'multitask' in multitask_name
    if multitask_name.endswith('megaverse8'):
        tasks = MEGAVERSE8
    elif multitask_name.endswith('obstacles'):
        tasks = OBSTACLES_MULTITASK
    else:
        raise NotImplementedError()

    scenario_idx = task_idx % len(tasks)
    scenario = tasks[scenario_idx]
    print('Multi-task, scenario', scenario_idx, scenario)
    return MegaverseEnv(scenario, num_envs, num_agents_per_env, num_simulation_threads, use_vulkan, params)


class MegaverseEnv(gym.Env):
    def __init__(self, scenario_name, num_envs, num_agents_per_env, num_simulation_threads, use_vulkan=False, params=None):
        scenario_name = scenario_name.casefold()
        self.scenario_name = scenario_name

        self.is_multiagent = True

        set_megaverse_log_level(2)

        self.img_w = 128
        self.img_h = 72
        self.channels = 3

        self.use_vulkan = use_vulkan

        # total number of simulated agents
        self.num_agents = num_envs * num_agents_per_env
        self.num_envs = num_envs
        self.num_agents_per_env = num_agents_per_env

        float_params = {}
        if params is not None:
            for k, v in params.items():
                if isinstance(v, float):
                    float_params[k] = v
                else:
                    raise Exception('Params of type %r not supported', type(v))

        # float_params['episodeLengthSec'] = 1.0

        self.env = MegaverseGym(
            self.scenario_name,
            self.img_w, self.img_h, num_envs, num_agents_per_env, num_simulation_threads, use_vulkan, float_params,
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

    def step(self, actions):
        action_idx = 0
        for env_i in range(self.num_envs):
            for agent_i in range(self.num_agents_per_env):
                self.env.set_actions(env_i, agent_i, actions[action_idx])
                action_idx += 1

        self.env.step()

        dones, infos = [], []

        agent_i = 0
        for env_i in range(self.num_envs):
            done = self.env.is_done(env_i)  # currently no individual done per agent
            dones.extend([done for _ in range(self.num_agents_per_env)])
            if done:
                infos.extend([dict(true_reward=float(self.env.true_objective(env_i, j))) for j in range(self.num_agents_per_env)])
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
        self.env.draw_overview()

        self.env.draw_hires()

        rows = []
        for env_i in range(self.num_envs):
            obs = [self.convert_obs(self.env.get_hires_observation(env_i, i)) for i in range(self.num_agents_per_env)]
            obs_concat = np.concatenate(obs, axis=1)
            rows.append(obs_concat)

        obs_final = np.concatenate(rows, axis=0)
        cv2.imshow(f'agent_{id(self)}', obs_final)
        cv2.waitKey(1)
        return obs_final

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
