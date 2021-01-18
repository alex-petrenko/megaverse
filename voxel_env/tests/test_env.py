import copy
import os
import time

import numpy as np

from unittest import TestCase

from voxel_env.voxel_env_gym import VoxelEnv


def sample_actions(e):
    return [e.action_space.sample() for _ in range(e.num_agents)]


def make_env(num_envs, num_agents_per_env, num_simulation_threads, use_vulkan=False, params=None):
    """Making env with a default scenario name."""
    return VoxelEnv('ObstaclesEasy', num_envs, num_agents_per_env, num_simulation_threads, use_vulkan, params)


class TestEnv(TestCase):
    def test_env(self):
        e = make_env(num_envs=1, num_agents_per_env=1, num_simulation_threads=1)
        o = e.reset()
        o = e.step(sample_actions(e))
        e.close()

    def test_env_close_immediately(self):
        e = make_env(1, 1, 1)
        e.close()

    def test_two_envs_same_process(self):
        e1 = make_env(1, 1, 1)
        e2 = make_env(1, 1, 1)

        e1.reset()
        e2.reset()

        e1.close()
        e2.close()

    def test_seeds(self):
        e1 = make_env(1, 1, 1)
        e1.seed(42)
        e2 = make_env(1, 1, 1)
        e2.seed(42)

        obs1 = e1.reset()
        obs2 = e2.reset()

        self.assertTrue(np.array_equal(obs1, obs2))

        # after this we have randomness due to physics?

    def rendering(self, use_vulkan, episode_length_sec=60.0):
        params = {'episodeLengthSec': episode_length_sec}

        e1 = make_env(num_envs=2, num_agents_per_env=2, num_simulation_threads=2, use_vulkan=use_vulkan, params=params)
        e2 = make_env(num_envs=1, num_agents_per_env=1, num_simulation_threads=1, use_vulkan=use_vulkan, params=params)

        e1.reset()
        e2.reset()

        e1.render()
        e2.render()

        for i in range(100):
            e1.step(sample_actions(e1))
            e1.render()
            e2.step(sample_actions(e2))
            e2.render()

        e1.close()
        e2.close()

    def test_render(self):
        self.rendering(use_vulkan=False)

    def test_render_vulkan(self):
        self.rendering(use_vulkan=True)

    def test_render_reset(self):
        self.rendering(use_vulkan=False, episode_length_sec=1.0)

    def test_render_vulkan_reset(self):
        self.rendering(use_vulkan=True, episode_length_sec=1.0)

    @staticmethod
    def performance_num_envs(n, n_steps=5000):
        print(f'Performance {n} {n_steps}')
        envs = [make_env(1, 1, 1, use_vulkan=True) for _ in range(n)]
        for e in envs:
            e.seed(42)
            e.reset()

        total_reward = 0.0
        actions = sample_actions(envs[0])
        start = time.time()
        for step in range(n_steps):
            for i, e in enumerate(envs):
                obs, rew, dones, infos = e.step(actions)
                total_reward += sum(rew)
                if all(dones):
                    print(f'Episode boundary env {i}')
        end = time.time()
        elapsed = end - start
        fps = envs[0].num_agents * n * n_steps / elapsed
        print(f'Time {elapsed:.3f}, fps: {fps:.1f}, total reward: {total_reward:.3f}')

        for e in envs:
            e.close()
        return fps

    def test_performance(self):
        fps1 = self.performance_num_envs(1)
        fps2 = self.performance_num_envs(2)
        fps4 = self.performance_num_envs(4)

        # print(fps1, fps2, fps4)

    def test_reward_shaping(self):
        e = VoxelEnv('TowerBuilding', num_envs=3, num_agents_per_env=2, num_simulation_threads=2, use_vulkan=True)
        default_reward_shaping = e.get_default_reward_shaping()
        self.assertEqual(default_reward_shaping, e.get_current_reward_shaping(0))
        self.assertEqual(default_reward_shaping, e.get_current_reward_shaping(1))
        self.assertEqual(default_reward_shaping, e.get_current_reward_shaping(2))
        self.assertEqual(default_reward_shaping, e.get_current_reward_shaping(5))

        new_reward_shaping = copy.deepcopy(default_reward_shaping)
        for k, v in new_reward_shaping.items():
            new_reward_shaping[k] = v * 3
        e.set_reward_shaping(new_reward_shaping, 3)

        self.assertEqual(default_reward_shaping, e.get_current_reward_shaping(0))
        self.assertEqual(default_reward_shaping, e.get_current_reward_shaping(1))
        self.assertNotEqual(default_reward_shaping, e.get_current_reward_shaping(3))

        e.close()

    def test_memleak(self):
        def mem_usage_kb():
            import psutil
            process = psutil.Process(os.getpid())
            return process.memory_info().rss / 1024

        params = {'episodeLengthSec': 0.1}
        e = VoxelEnv('Rearrange', num_envs=32, num_agents_per_env=1, num_simulation_threads=1, use_vulkan=True, params=params)
        e.reset()

        orig_mem_usage = mem_usage_kb()

        for i in range(1000):
            print('Mem difference: ', mem_usage_kb() - orig_mem_usage, 'kb')
            e.step(sample_actions(e))

        print('Final mem difference: ', mem_usage_kb() - orig_mem_usage, 'kb')

        e.close()
