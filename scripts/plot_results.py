#!/usr/bin/env python3
"""
plot_results.py

Reads logged flight CSVs from data/ and produces plots:
landing accuracy distribution, altitude/velocity profile 
over time, fuel usage, etc.

Usage:
    python scripts/plot_results.py --input data/landing_log.csv
"""

import argparse
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


def main():
    parser = argparse.ArgumentParser(description="Plot descent guidance results")
    parser.add_argument("--input", type=str, required=True, help="Path to CSV log")
    args = parser.parse_args()

    # TODO once there is real logged data:
    # import pandas as pd
    # import matplotlib.pyplot as plt
    # df = pd.read_csv(args.input)
    # -- altitude vs. time plot
    # -- vertical speed vs. time plot
    # -- histogram of landing error across runs
    # -- save figures to docs/images/

    raise NotImplementedError("Implement once there is CSV data to plot.")


if __name__ == "__main__":
    main()
