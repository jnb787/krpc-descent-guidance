# Technical Writeup: Autonomous Powered-Descent Guidance

## 1. Motivation

_(Why this project -- tie to aerospace/GNC interest, ROAR Academy background.)_

## 2. Problem Statement

_(What autonomous landing guidance needs to solve: timing the braking
burn, hitting zero velocity at the surface, correcting horizontal drift.)_

## 3. Suicide-Burn Guidance Derivation

_(Walk through the kinematics: v^2 = v0^2 + 2*a*d, and how you arrived
at `suicide_burn_altitude()`. Include the actual math, not just the code.)_

## 4. System Architecture

_(Brief description of the guidance/, scripts/, mission phases. Maybe
a simple diagram.)_

## 5. Controller Design

_(PID tuning process -- what gains you started with, how you tuned
them, what problems you ran into like overshoot or oscillation.)_

## 6. Results

_(Landing accuracy across N test runs, fuel usage, plots. This is
where your SMART goal's "Measurable" criteria get reported.)_

## 7. Challenges and What I'd Do Differently

_(Honest reflection -- this is often the most compelling part for
an application reader.)_

## 8. References

_(kRPC docs, any papers/articles you used to understand suicide-burn
guidance or SpaceX's approach, ROAR Academy materials.)_
