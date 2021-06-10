from sample_factory.runner.run_description import RunDescription, Experiment, ParamGrid

_params = ParamGrid([
    ('env', ['megaverse_rearrange', 'megaverse_collect', 'megaverse_obstaclesEasy', 'megaverse_hexMemory']),
    ('megaverse_num_simulation_threads', [1]),
    ('rnn_num_layers', [2]),
])

_experiment = Experiment(
    'megaverse_pbt',
    'python -m megaverse_rl.train --train_for_seconds=360000000 --algo=APPO --gamma=0.997 --use_rnn=True --num_workers=12 --num_envs_per_worker=2 --ppo_epochs=1 --rollout=32 --recurrence=32 --batch_size=2048 --actor_worker_gpus 0 --num_policies=1 --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --megaverse_num_envs_per_instance=36 --megaverse_num_agents_per_env=1 --megaverse_num_simulation_threads=1 --megaverse_use_vulkan=True --policy_workers_per_policy=2 --learner_main_loop_num_cores=4',
    _params.generate_params(randomize=False),
)

RUN_DESCRIPTION = RunDescription('megaverse_v114_env_v52', experiments=[_experiment])
