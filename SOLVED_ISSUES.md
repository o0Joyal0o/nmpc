# Solved Issues Documentation

This document tracks all issues solved in the NMPC Swarm Demo project. **Do not undo these fixes.**

---

## Issue 1: Formation Not Working (Both Robots Not Going to Goal)
**Date:** 2026-10-24
**Root Cause:** `w_avoid=50` >> `w_pos=10` - obstacle avoidance dominated position tracking
**Fix:** Rebalanced costs in `demo.launch`
- `w_pos`: 10 → 20
- `w_avoid`: 50 → 8
- `formation_offset`: 0.35 → 0.5
**Files:** `demo.launch`, `central_controller.py`

---

## Issue 2: Snake/Wiggle Movement (Wheels Alternating Brake)
**Date:** 2026-10-24
**Root Cause:** Heading cost `1-cos(Δθ)` is symmetric - solver randomly picks left/right turn each iteration
**Fix:** 
- Heading cost → `sin²(Δθ/2)` (breaks left/right symmetry, gradient points to shortest rotation)
- Added angular acceleration penalty `R_domega * (ω_k - ω_{k-1})²`
- Increased control rate: 10 → 20 Hz, dt: 0.1 → 0.05, horizon: 10 → 20
**Files:** `central_controller.py`, `demo.launch`

---

## Issue 3: Robot Freezes at Obstacles (Over-Cautious)
**Date:** 2026-10-24
**Root Cause:** Velocity penalty `v²` makes robot want to stop; quadratic avoidance penalty only activates when very close
**Fix:**
- Added cruise control: `w_cruise * (v_max - v)²` - penalizes NOT going at max speed
- Changed avoidance to inverse barrier: `w / (dist²/d_min² + ε)` - repulsion → ∞ as dist → 0
- `R_v`: 0.5 → 0.05 (tiny velocity regularization)
**Files:** `central_controller.py`, `demo.launch`

---

## Issue 4: IPOPT Failures → Robot Stops
**Date:** 2026-10-24
**Root Cause:** Single solve attempt, no fallback on saddle points (left vs right avoidance)
**Fix:** 3-attempt fallback:
1. Warm-start (full trajectory from previous iteration)
2. Cold-start (zero controls, propagate from current state)
3. Safe stop (return 0, 0)
**Files:** `central_controller.py`

---

## Issue 5: Slow Turning / Wheel Slip
**Date:** 2026-10-24
**Root Cause:** Caster friction too low, drive wheels lack grip, wrong wheel separation, low torque, low update rate
**Fix (URDF `box_bot.urdf.xacro`):**
- Caster: `mu=0, slip=1` (free rotation)
- Drive wheels: `mu=1.0, fdir1=1 0 0, kp=1e6, kd=100` (high grip, no sideways slip)
- Wheel separation: 0.2 → 0.35 (matches joint positions)
- Wheel torque: 5 → 20 Nm
- Wheel acceleration: 0 → 10 rad/s²
- Plugin update rate: 10 → 100 Hz
- Base link inertia: corrected box formula values
**Files:** `box_bot.urdf.xacro`

---

## Issue 6: Reverse Speed Too Slow
**Date:** 2026-10-24
**Root Cause:** `v_min=-0.2` vs `v_max=1.0`
**Fix:** `v_min: -0.2 → -0.8` (80% of forward speed)
**Files:** `demo.launch`

---

## Issue 7: Circling at Goal Position
**Date:** 2026-10-24
**Root Cause:** No arrival condition - NMPC keeps running, overshoots, inter-robot avoidance pushes apart
**Fix (Per-robot, Nav2-style):**
- Per-robot `robot_arrived[name]` flag (not global both_arrived)
- Two-tier tolerance: fine (0.15m) + coarse (0.30m hysteresis)
- Velocity check: `speed < 0.05 m/s`
- Hold time: 0.5s in fine zone
- Hysteresis: once arrived, stays arrived until leaves coarse zone
- Stronger terminal cost: 5× position/heading + terminal velocity penalty
**Files:** `central_controller.py`, `demo.launch`

---

## Issue 8: Slow Obstacle Avoidance Reflexes
**Date:** 2026-10-24
**Root Cause:** 20Hz loop, 1s horizon, static obstacle positions over horizon
**Fix:**
- Control rate: 20 → 30 Hz
- dt: 0.05 → 0.033s
- Horizon: 20 → 15 (0.5s prediction)
- `w_avoid`: 12 → 15
- `w_max`: 2.0 → 2.5
- **Predictive obstacle positions**: `ox_pred = ox_curr + vx_obs * (k * dt)`
- Time-weighted avoidance: higher weight for near-term predictions
- Obstacle velocity estimation from `/moving_obstacles` (fixed index order)
**Files:** `central_controller.py`, `demo.launch`

---

## Issue 10: Robot Not Stopping at Goal (Cruise Control Prevents Stopping)
**Date:** 2026-10-24
**Status:** FIXED
**Root Cause:** Cruise control `w_cruise * (v_max - v)²` forces robot to maintain max speed (1 m/s) even at goal position. Robot overshoots, turns around, comes back, overshoots again - never slowing down enough to trigger arrival detection.

**Symptoms from logs:**
- `dist=0.22→0.9` but `in_fine=False` always
- `speed=1.0-2.6 m/s` near goal (cruise control forcing max speed)
- `angle=2.9 rad` (180° wrong) - robot spins/overshoots

**Fix:**
1. **Distance-scaled cruise control**: `cruise_scale = min(1.0, dist²_goal / 4.0)` - full cruise at >2m, zero at goal
2. **Increased arrival tolerances**: `arrival_dist: 0.15→0.25m`, `arrival_angle: 0.2→0.35 rad`
3. Added comprehensive debug logging to diagnose future issues

**Files:** `central_controller.py`, `demo.launch`