from unittest import TestCase

from sample_factory.algorithms.utils.arguments import default_cfg
from sample_factory.envs.create_env import create_env
from sample_factory.utils.utils import log

from megaverse_rl.megaverse_utils import register_env


class TestMegaverse(TestCase):
    def test_megaverse(self):
        register_env()

        env_name = 'megaverse_Sokoban'
        env = create_env(env_name, cfg=default_cfg(env=env_name))
        log.info('Env action space: %r', env.action_space)
        log.info('Env obs space: %r', env.observation_space)

        env.reset()
        total_rew = 0
        for i in range(1000):
            obs, rew, done, info = env.step([env.action_space.sample() for _ in range(env.num_agents)])
            total_rew += sum(rew)

        log.info('Total rew: %.3f', total_rew)
