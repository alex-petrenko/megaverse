from sample_factory.launcher.run_description import RunDescription, Experiment, ParamGrid

NUM_WORKERS_MEGAVERSE = 48  # typically num logical cores / 2, limited by the num of available Vulkan contexts
TIMEOUT_SECONDS = 180
SAMPLER_GPUS = '0 1 2 3 4 5 6 7'  # replace with '0 1 2 3 4 5 6 7' for 8-GPU server

_megaverse_cli = f'python -m sample_factory.run_algorithm --algo=DUMMY_SAMPLER --num_workers={NUM_WORKERS_MEGAVERSE} --num_envs_per_worker=1 --experiment=benchmark --sampler_worker_gpus {SAMPLER_GPUS} --megaverse_num_envs_per_instance=64 --megaverse_num_agents_per_env=2 --megaverse_num_simulation_threads=2 --timeout_seconds={TIMEOUT_SECONDS}'

_params_megaverse = ParamGrid([
    ('env', ['TowerBuilding', 'ObstaclesEasy', 'ObstaclesHard', 'Collect', 'Sokoban', 'HexMemory', 'HexExplore', 'Rearrange']),
    ('megaverse_use_vulkan', [True]),
])

_experiment_megaverse = Experiment(
    'benchmark_megaverse_8',
    _megaverse_cli,
    _params_megaverse.generate_params(randomize=False),
)


RUN_DESCRIPTION = RunDescription('megaverse_bench_sampling_all_envs', experiments=[_experiment_megaverse])
