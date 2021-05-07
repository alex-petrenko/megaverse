import sys

from sample_factory.algorithms.appo.enjoy_appo import enjoy
from sample_factory.algorithms.utils.arguments import parse_args

from megaverse_rl.voxel_env_utils import register_env


def main():
    """Script entry point."""
    register_env()
    cfg = parse_args(evaluation=True)
    status = enjoy(cfg)
    return status


if __name__ == '__main__':
    sys.exit(main())
