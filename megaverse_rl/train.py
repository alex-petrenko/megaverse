"""
Main script for training agents with SampleFactory.

"""

import sys

from sample_factory.algorithms.utils.arguments import parse_args
from sample_factory.run_algorithm import run_algorithm

from megaverse_rl.megaverse_utils import register_env


def main():
    """Script entry point."""
    register_env()
    cfg = parse_args(evaluation=False)
    status = run_algorithm(cfg)
    return status


if __name__ == '__main__':
    sys.exit(main())
