"""
Measure pure sampling throughput.

"""

import sys

from sample_factory.algorithms.utils.arguments import arg_parser, postprocess_args
from sample_factory.run_algorithm import run_algorithm
from sample_factory.utils.get_available_gpus import get_gpus_without_triggering_pytorch_cuda_initialization
from sample_factory.utils.utils import log

from megaverse_rl.voxel_env_utils import register_env


def main():
    """Script entry point."""
    gpus = get_gpus_without_triggering_pytorch_cuda_initialization()
    gpus = gpus.strip().split(',')
    gpus = [int(g) for g in gpus]

    if len(gpus) <= 0:
        log.error('Sampling benchmark requires at least one GPU')
    else:
        log.debug('Have %d GPUs (%r)', len(gpus), gpus)

    register_env()

    argv = sys.argv[1:]
    argv.append('--algo=DUMMY_SAMPLER')

    parser = arg_parser(argv)
    parser.set_defaults(
        sampler_worker_gpus=gpus,
        num_workers=len(gpus) * 10,
        voxel_num_envs_per_instance=32,
        voxel_num_agents_per_env=4,
        voxel_num_simulation_threads=2,
    )

    # parse all the arguments (algo, env, and optionally evaluation)
    cfg = parser.parse_args(argv)
    cfg = postprocess_args(cfg, argv, parser)

    status = run_algorithm(cfg)
    return status


if __name__ == '__main__':
    sys.exit(main())
