# DSIQ Native — Verified Two-Solver Actuator Engineering Milestone

**Date:** 23 November 2026  
**Source:** Real GitHub Actions CPU Bullet 3.2.7 / MuJoCo 3.17.0 Linux Python 3.11 job, source-authored reference gantry. No physical machinery or OEM model was measured.

- [CI evidence run with full 100-step traces](https://github.com/Niroop-H/DSIQ-Native/actions/runs/45114534074)
- Artifact: `dsiq-two-real-physics-solvers`, including `full-motor-position-velocity-trace.json`, `actual-bullet-mujoco-comparison.json` and MuJoCo conformance.
- To execute the same API: `python -m pip install -e ".[mujoco]"`, then `python -m dsiq_native physics-motion-trace --steps 100 --sample-every 5 --tolerance .01 --require-match`.

## Test conditions and independent measurements

| Quantity | Real Bullet CPU | Real MuJoCo CPU |
| --- | ---: | ---: |
| Source fixture | Same validated original URDF SHA256 | Same validated original URDF SHA256 |
| Target actuator_x | 0.200 m | 0.200 m |
| Integration timestep | 1/240 s | 1/240 s |
| Steps | 100 | 100 |
| Final X position | 0.199725792365 m | 0.199980968423 m |
| Final X velocity | 0.007312204 m/s | 0.012831589 m/s |
| Peak measured X speed | 0.800000 m/s | 0.799477 m/s |
| Source speed limit | 0.8 m/s | 0.8 m/s |
| Peak reported generalized motor effort | 250 N | 250 N |
| Source effort limit | 250 N | 250 N |
| Verified source travel | 0..1.2 m | 0..1.2 m |

- Final source-matched position residual: **0.000255176 m (0.255 mm)**.
- Sampled peak inter-solver position residual: **0.009412845 m (9.413 mm)**.
- Sampled position RMSE: **0.005455600 m (5.456 mm)**.
- Source speed, effort and travel envelopes: **PASS** on both solvers in this test.
- More than 100 million CPU instructions may execute internally, but this evidence covers only the *explicit measured 100 steps*, not any invented performance claims.
- On the separate **uncontrolled freefall** 72-step fixture, both engines agreed in z and vz to the printed numerical precision. Both differed from continuous-time analytic z by about **6.131 mm** because of integration.

## Controller change and why it matters

The initial MuJoCo reference controller `600 × position_error - 48 × velocity` (clamped to source effort) overshot the example 0.200 m target to approximately **0.27491 m**, whereas Bullet reached approximately **0.19973 m**. The original 75.2 mm difference was a real failure to match the control response.

The revised MuJoCo reference drive uses a cascaded controller:
- Position-to-commanded-speed gain `12 s^-1`.
- Commanded speed bounded to the **actual URDF maximum** for each axis.
- Prismatic velocity feedback reference gain `450 N·s/m`, revolute reference gain `50 N·m·s/rad`.
- Generalized control force/torque clamped to the **URDF effort limit**.
- DSIQ records true MuJoCo `MjData` and Bullet `getJointState` feedback, not fabricated or averaged solver state.

These are engineering **example controller coefficients**, not physically calibrated OEM servo coefficients. A similar end state does not establish matching acceleration, motor loading, contact stability or closed-loop control behavior across different machines.

## Updated CI acceptance

- CPU backend contract tests exercise both real engines, independent sources and no silent fallbacks.
- `physics-cross-backend-check --tolerance 0.01` must pass at the final step.
- `physics-motion-trace --steps 100 --sample-every 5 --tolerance .01 --require-match` must pass.
- CI independently verifies **sampled peak position discrepancy ≤ 0.01 m** and true source **speed, effort and travel** envelopes.
- Independent analytic freefall comparison remains mandatory.
- Invalid/overtravel input and source-URDF-specific target limits are negative-tested.
- The tool still reports no certification, no industrial hardware control and no cross-solver equivalence for arbitrary physics models.

## Next engineering tests before broader industrial claims

1. Match/tune explicit actuator control laws on both engines using **independently measured motor and transmission parameters**, not merely adjusting a gain to match an arbitrary final point.
2. Compare full 240 Hz traces over multiple target magnitudes, both travel directions, load cases and stationary hold periods. Record position, velocity, effort, acceleration and jerk envelopes for each.
3. Test contacts and possible slip, friction, gravity counterbalance, loop stability, controller startup from nonzero pose, numeric conditioning and solver timestep convergence.
4. Compare against a physical instrumented reference with traceable uncertainty, calibrated mass/inertia, manufacturer speed/effort specifications and signed independent sign-off before claiming an industrial digital twin.
5. Retain full artifacts and limits by commit SHA. Do not change acceptance thresholds merely to make CI green.

**Sources:** [MuJoCo modelling and control-limit semantics](https://mujoco.readthedocs.io/en/latest/modeling.html), [MuJoCo `mj_step` and `qfrc_applied` timing](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html); self-authored DSIQ source CAD/reference model and committed acceptance suites.

The overall DSIQ 42/100 previously assigned qualification baseline is **not** automatically raised by this software milestone.
