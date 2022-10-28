from sample_factory.launcher.run_description import RunDescription, Experiment, ParamGrid

_params = ParamGrid([
    ('env', ['multitask_megaverse8']),
    ('seed', [11111, 22222, 33333, 44444, 55555]),
])

_cli = 'python -m megaverse_rl.train_megaverse --train_for_seconds=360000000 --train_for_env_steps=2000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers=12 --num_envs_per_worker=2 --num_epochs=1 --rollout=32 --recurrence=32 --batch_size=2048 --actor_worker_gpus 0 --num_policies=1 --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --megaverse_num_simulation_threads=1 --megaverse_use_vulkan=True --policy_workers_per_policy=2 --reward_clip=30 --pbt_mix_policies_in_one_env=False'

EXPERIMENT_1AGENT = Experiment(
    'megaverse_multitask_obs',
    _cli + ' --megaverse_num_envs_per_instance=36 --megaverse_num_agents_per_env=1',
    _params.generate_params(randomize=False),
)

RUN_DESCRIPTION = RunDescription('megaverse_v115_multitask8_v55', experiments=[EXPERIMENT_1AGENT])
