from sample_factory.launcher.run_description import RunDescription, Experiment, ParamGrid

NUM_WORKERS = 20  # typically num logical cores
NUM_WORKERS_MEGAVERSE = 10  # typically num logical cores / 2, limited by the num of available Vulkan contexts
TIMEOUT_SECONDS = 180
ACTOR_GPUS = '0'  # replace with '0 1 2 3 4 5 6 7' for 8-GPU server
NUM_POLICIES = 1

_basic_cli = f'python -m megaverse_rl.train_megaverse --train_for_seconds={TIMEOUT_SECONDS} --train_for_env_steps=20000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers={NUM_WORKERS} --num_envs_per_worker=16 --num_epochs=1 --rollout=32 --recurrence=32 --batch_size=2048 --num_policies={NUM_POLICIES} --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --policy_workers_per_policy=2 --learner_main_loop_num_cores=4 --reward_clip=30'

_params_basic_envs = ParamGrid([
    ('env', ['doom_benchmark', 'atari_breakout', 'dmlab_benchmark']),
])

_experiment_basic_envs = Experiment(
    'benchmark_basic_envs',
    _basic_cli,
    _params_basic_envs.generate_params(randomize=False),
)

_megaverse_cli = f'python -m megaverse_rl.train_megaverse --train_for_seconds={TIMEOUT_SECONDS} --train_for_env_steps=20000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers={NUM_WORKERS_MEGAVERSE} --num_envs_per_worker=2 --num_epochs=1 --rollout=32 --recurrence=32 --batch_size=2048 --actor_worker_gpus {ACTOR_GPUS} --num_policies={NUM_POLICIES} --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --megaverse_num_simulation_threads=2 --megaverse_use_vulkan=True --policy_workers_per_policy=2 --learner_main_loop_num_cores=4 --reward_clip=30 --megaverse_num_envs_per_instance=36 --megaverse_num_agents_per_env=1 --pbt_mix_policies_in_one_env=False'
_params_megaverse = ParamGrid([
    ('env', ['megaverse_obstacleshard']),
    ('megaverse_use_vulkan', [True, False]),
])

_experiment_megaverse = Experiment(
    'benchmark_megaverse',
    _megaverse_cli,
    _params_megaverse.generate_params(randomize=False),
)


RUN_DESCRIPTION = RunDescription('megaverse_train_benchmark', experiments=[_experiment_basic_envs, _experiment_megaverse])
