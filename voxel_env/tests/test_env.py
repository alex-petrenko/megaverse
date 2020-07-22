from unittest import TestCase

from voxel_env_gym import VoxelEnv


class TestEnv(TestCase):
    def test_env(self):
        e = VoxelEnv()
        o = e.reset()
        o = e.step(0)
        e.close()

    def test_two_envs_same_process(self):
        e1 = VoxelEnv()
        e2 = VoxelEnv()

        e1.reset()
        e2.reset()

        e1.close()
        e2.close()

    def test_render(self):
        e1 = VoxelEnv()
        e2 = VoxelEnv()

        obs1 = e1.reset()
        obs2 = e2.reset()

        e1.close()
        e2.close()
