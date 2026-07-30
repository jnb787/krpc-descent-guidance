"""
flight_log.py

CSV flight-data logger. Used by both the manual logging script and
(later) the autonomous mission loop, so both produce identically
formatted data for plot_results.py.
"""

import csv
import os
from datetime import datetime


class FlightLogger:
    """Writes one CSV row per guidance-loop tick.

    Usage:
        logger = FlightLogger("data", prefix="manual_flight",
                              fields=["time", "altitude", "vertical_speed"])
        logger.log({"time": 1.5, "altitude": 1200.0, "vertical_speed": -42.1})
        ...
        logger.close()
    """

    def __init__(self, directory: str, prefix: str, fields: list[str]):
        os.makedirs(directory, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        self.path = os.path.join(directory, f"{prefix}_{timestamp}.csv")
        self.fields = fields
        self._file = open(self.path, "w", newline="")
        self._writer = csv.DictWriter(self._file, fieldnames=fields)
        self._writer.writeheader()

    def log(self, row: dict) -> None:
        self._writer.writerow(row)
        self._file.flush()  # so data survives even if KSP/script crashes

    def close(self) -> None:
        self._file.close()


def write_summary(directory: str, results: dict,
                  filename: str = "landing_summary.csv") -> str:
    """Append one row per landing run to a cumulative summary CSV.

    Unlike FlightLogger (one file per run, one row per tick), this keeps
    every run in a single file, so plot_results.py can read landing error
    across all 20+ runs without globbing directories.

    Returns:
        The path written to.
    """
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, filename)
    fields = ["timestamp"] + list(results.keys())
    row = {"timestamp": datetime.now().strftime("%Y-%m-%d_%H%M%S"), **results}

    is_new = not os.path.isfile(path)
    with open(path, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if is_new:
            writer.writeheader()
        writer.writerow(row)
    return path