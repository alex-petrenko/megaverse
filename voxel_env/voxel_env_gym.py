import gym

# noinspection PyUnresolvedReferences
from extension.voxel_env import VoxelEnvGym


class VoxelEnv(gym.Env):
    def __init__(self):
        self.env = None
        self.initialized = False

    def ensure_initialized(self):
        if self.initialized:
            return

        self.env = VoxelEnvGym(128, 72)
        self.initialized = True

    def reset(self):
        self.ensure_initialized()

        self.env.reset()
        obs = self.env.getObservation(0)
        return obs

    def step(self, action):
        self.ensure_initialized()

        self.env.step()
        obs = self.env.getObservation(0)
        return obs

    def render(self, mode='human'):
        pass

    def close(self):
        if self.env:
            self.env.close()

    def seed(self, seed=None):
        """TODO"""
        pass
