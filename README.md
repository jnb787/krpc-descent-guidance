# Autonomous Powered-Descent Guidance for KSP

https://github.com/user-attachments/assets/cca0fa91-44ea-4c42-8734-01e46c119813

An autonomous guidance system that lands a Kerbal Space Program rocket booster using real-time telemetry and closed-loop control. This project was inspired by SpaceX’s Falcon 9 powered descent booster recovery.

Built with [kRPC](https://krpc.github.io/krpc/), a mod that exposes KSP’s flight data and controls to external Python scripts.

## Status

System functional, awaiting full testing and writeup. Project started July 2026, target completion of all items mid October 2026. Created as a SMART-goal project for ROAR Academy to develop and apply control theory topics in Python.

## What it does

Vessel is guided through five autonomous phases:

1. **Deorbit** -- executes a burn to drop periapsis into Kerbin’s atmosphere
2. **Correct** -- executes a high altitude burn that adjusts trajectory towards the Kerbal Space Center
3. **Coast** -- monitors descent until it's time to begin braking
4. **Descent (suicide burn)** -- throttle and attitude controlled in real time to hit zero vertical velocity exactly at the surface, while nulling out horizontal drift toward the target
5. **Landed** -- touchdown, results logged

## Project structure

```
guidance/     core Python package (telemetry, vehicle, controllers, mission logic)
scripts/      runnable entry points
tests/        unit tests for the guidance math
data/         logged flight data from test landings
notebooks/    exploratory / derivation notebooks
docs/         technical writeup and images
craft/        .craft file of booster used for testing  
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

## Tests

The guidance math — PID control and the suicide-burn altitude calculation — is unit-tested independently of KSP. No game or kRPC connection required:

```bash
pytest tests/
```

Everything in `guidance/utils.py` and `guidance/controllers.py` is pure computation, which is what makes it testable in isolation. Mission logic and the kRPC wrappers are verified in-game.

## Usage

1. Launch KSP and start the kRPC server 
2. Load the craft from `craft/` into an ~80 km circular orbit, or a ballistic trajectory with ~80 km apoapsis 
3. Run:

```bash
python scripts/run_landing.py --lat -0.097207 --lon -74.557682
```

## Results


| Trial # | Landing Error (m) | Fuel Used (kg)|
| -------- | -------- | -------- |
| 1 | 2.89 | 29636 |
| 2 | 2.28 | 29623 |
| 3 | 5.29 | 29588 |
| 4 | 4.72 | 29599 |
| 5 | 5.09 | 29596 |
| 6 | 4.47 | 29600 |
| 7 | 4.51 | 29577 |
| 8 | 3.11 | 29602 |
| 9 | 4.62 | 29586 |
| 10 | 4.65 | 29599 |
| 11 | 4.48 | 29607 |
| 12 | 3.94 | 29595 |
| 13 | 4.17 | 29597 |
| 14 | 3.90 | 29586 |
| 15 | 3.86 | 29596 |
| 16 | 4.60 | 29614 |
| 17 | 3.66 | 29618 |
| 18 | 4.21 | 29623 |
| 19 | 3.05 | 29589 |
| 20 | 3.59 | 29620 |


## Background

This project extends autonomous-systems concepts from
[ROAR Academy](https://vive.berkeley.edu/) (UC Berkeley, Python / autonomous driving / ML-RL) into an aerospace guidance, navigation, and control (GNC) context.

See [docs/writeup.md](docs/writeup.md) for the full technical writeup.

## License

MIT

