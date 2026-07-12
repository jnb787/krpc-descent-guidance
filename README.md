# Autonomous Powered-Descent Guidance for KSP

An autonomous guidance system that lands a Kerbal Space Program rocket
using real-time telemetry and closed-loop control -- a simplified
version of what SpaceX's Falcon 9 booster does during a powered
landing.

Built via [kRPC](https://krpc.github.io/krpc/), a mod that exposes
KSP's flight data and controls to external Python scripts.

## Status

🚧 In progress. Project started July 2026, target completion ~October
2026, as a SMART-goal project for ROAR Academy / college application
portfolio.

## What it does

The guidance system flies a vessel through four autonomous phases:

1. **Deorbit** -- executes a burn to drop periapsis toward the landing target
2. **Coast** -- monitors descent until it's time to begin braking
3. **Powered descent (suicide burn)** -- throttle and attitude
   controlled in real time to hit zero vertical velocity exactly at
   the surface, while nulling out horizontal drift toward the target
4. **Landed** -- touchdown, results logged

## Project structure

```
guidance/     core Python package (telemetry, vehicle, controllers, mission logic)
scripts/      runnable entry points
tests/        unit tests for the guidance math
data/         logged flight data from test landings
notebooks/    exploratory / derivation notebooks
docs/         technical writeup and images
```

## Setup

```bash
conda create -n krpc-guidance python=3.11
conda activate krpc-guidance
pip install -r requirements.txt
```

You'll also need:
- Kerbal Space Program (KSP1, not KSP2)
- the [kRPC server mod](https://spacedock.info/mod/69/kRPC) installed in KSP

## Usage

1. Launch KSP, load a save with a vessel in orbit, start the kRPC server
2. Run:

```bash
python scripts/run_landing.py --lat -0.09720 --lon -74.55767
```

## Results

_(To be filled in as testing progresses: landing accuracy across N
runs, fuel efficiency, comparison plots.)_

## Background

This project extends autonomous-systems concepts from
[ROAR Academy](https://vive.berkeley.edu/) (UC Berkeley, Python /
autonomous driving / ML-RL) into an aerospace guidance, navigation,
and control (GNC) context.

See [docs/writeup.md](docs/writeup.md) for the full technical writeup,
including the suicide-burn guidance derivation.

## License

MIT
