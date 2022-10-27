from sample_factory.launcher.run_description import RunDescription

from megaverse_rl.runs.megaverse_base_experiments import EXPERIMENT_4AGENTS, EXPERIMENT_2AGENTS

RUN_DESCRIPTION = RunDescription('megaverse_v115_multi_agent_v55', experiments=[EXPERIMENT_2AGENTS, EXPERIMENT_4AGENTS])
