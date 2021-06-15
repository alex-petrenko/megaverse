#!/bin/bash
source activate sample-factory
cd megaverse
python -m megaverse_rl.train --train_for_seconds=360000000 --train_for_env_steps=2000000000 --algo=APPO --gamma=0.997 --use_rnn=True --rnn_num_layers=2 --num_workers=12 --num_envs_per_worker=2 --ppo_epochs=1 --rollout=32 --recurrence=32 --batch_size=2048 --actor_worker_gpus 0 --num_policies=1 --with_pbt=False --max_grad_norm=0.0 --exploration_loss=symmetric_kl --exploration_loss_coeff=0.001 --megaverse_num_simulation_threads=1 --megaverse_use_vulkan=False --policy_workers_per_policy=2 --learner_main_loop_num_cores=1 --reward_clip=30 --env=megaverse_TowerBuilding --experiment=test_cli
# python -m unittest
