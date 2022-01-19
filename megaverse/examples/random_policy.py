import numpy as np
from megaverse.megaverse_env import MegaverseEnv


env = MegaverseEnv(
    'ObstaclesHard',
    num_envs=2, num_agents_per_env=2,
    num_simulation_threads=4, use_vulkan=True,
    params={},
)
env.reset()

while True:
    actions = [env.action_space.sample() for _ in range(env.num_agents)]
    obs, rewards, dones, infos = env.step(actions)
    if np.any(dones):
        break

    env.render()

env.close()
