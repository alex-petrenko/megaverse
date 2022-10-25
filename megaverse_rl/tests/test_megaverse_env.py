from unittest import TestCase

from sample_factory.envs.create_env import create_env
from sample_factory.utils.utils import log

from megaverse_rl.train_megaverse import register_megaverse_components, parse_megaverse_args


class TestMegaverse(TestCase):
    def test_megaverse(self):
        register_megaverse_components()

        cfg = parse_megaverse_args(['--algo=APPO', '--env=Sokoban', '--experiment=test_megaverse'])

        env = create_env(cfg.env, cfg=cfg)
        log.info('Env action space: %r', env.action_space)
        log.info('Env obs space: %r', env.observation_space)

        env.reset()
        total_rew = 0
        for i in range(1000):
            obs, rew, terminated, truncated, info = env.step([env.action_space.sample() for _ in range(env.num_agents)])
            total_rew += sum(rew)

        log.info('Total rew: %.3f', total_rew)
