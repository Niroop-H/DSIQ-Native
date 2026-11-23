# DSIQ Native — Evidence-first engineering qualification milestone (2026-11)

**Scope:** Niroop-H/DSIQ-Native, main. **Goal:** build a commercially credible
offline-first Windows + OpenUSD engineering workcell platform, with measured
robot motion, source-linked geometry and independently reviewable evidence.

**No numeric rescoring:** 42/100 is the existing provisional planning baseline.
Source-code improvements alone do not establish OEM calibration, physical
functional safety, commercial compliance, or hardware performance. Release to
an industrial safety-relevant environment remains blocked until external
review and site acceptance.

## Implemented in this milestone

| Engineering subsystem | Implemented checks | Evidence boundary |
|---|---|---|
| URDF motor-program admission | Match current imported robot identity, joint indices, URDF source motor velocities, measured initial posture, true fixed 240 Hz sample count, and every quintic trajectory sample | Prevents accidental or maliciously altered in-process trajectory objects; cannot authenticate an OEM controller |
| Runtime simulation motion monitor | Post-step Bullet motor velocity/effort, joint travel, optional joint tracking residual threshold and optional proximity to external obstacles. Stop issuing future targets and request zero-velocity motor control when a fault is observed | The first violating step has already occurred; no self-collision qualification, CCD or safety-rated stop |
| Measured physical validation | Require complete simulated-joint coverage, consistent m/rad units, finite measurements, uninterrupted simulation report. Optionally enforce conservative residual + instrument standard uncertainty margin | Inputs are operator-supplied, not independently authenticated or time-synchronized. Sim uncertainty not included |
| USD assembly/geometry profile | Read-only bounded scan of composed OpenUSD meshes, time-varying exclusions, exact local positions/topology reuse candidates, stage SHA-256 before/after and no destructive edits | No instance/merge safety, no GPU VRAM savings, no material or physics fidelity implied |

### Runnable commands

~~~
python -m pip install -e ".[robotics]"
python -m dsiq_native robot-urdf-motion machine.manifest.json --target actuator_x=0.03 --motion-limit-file reviewed-motion-limits.json --require-reviewed-limits --max-runtime-tracking-error 0.02 > simulation-motion.json

python -m dsiq_native physical-compare simulation-motion.json external-measurements.json --tolerance 0.005 --require-uncertainty --coverage-factor 2 > comparison.json

python -m dsiq_native usd-scene-profile real-workcell.usda > usd-scene-profile.json
~~~

These are examples of command *shape*, not validated engineering thresholds.
Motion-limit sources, tracking thresholds, tolerance and uncertainty must be
selected by a qualified engineer for a named instrument, machine and use case.

**Input contract for strict external uncertainty analysis**:
The existing \`dsiq-external-joint-measurements-v1\` JSON allows an optional
\`joint_units\` map (one \`m\` or \`rad\` value for every simulated joint)
and each observation can carry \`standard_uncertainty\` (one standard
deviation, same unit as measured position). In strict mode every observation
must have this finite nonnegative value. For each joint the software checks
\`max(abs(predicted - observed) + k * u) <= tolerance\`, where \`k\` is
the supplied coverage multiplier. This *conservative comparison rule*
does not constitute a formal confidence interval: it does not include
simulation uncertainty, correlated instrumentation errors or temporal
alignment uncertainty, nor does it establish calibration traceability.
Without original measured physical data the 0/10 independent validation
baseline must remain unchanged.

## Governing design principles from source references

1. **Isaac Lab 3.0 actuator model documentation** distinguishes drive
   effort, velocity constraints and controller execution mechanisms; a
   kinematically bounded quintic reference is not a validated motor/drive
   dynamic model. https://docs.nvidia.com/isaaclab/3.0/concepts/actuators
2. **Isaac Lab sim-to-real guidance (Oct 2026)** emphasizes domain shift from
   friction, contact, sensor, timing and actuator response. DSIQ therefore
   requires measured actuator and sensor parameters before transfer claims.
   https://docs.nvidia.com/learning/physical-ai/getting-started-with-isaac-lab/2026-10.0/robot-learning-fundamentals/08-simulation-to-reality.html
3. **NVIDIA Data Adapter Deduplication, Pivot and Merge Static Meshes**
   (user-supplied technical references) detail hashing, transform stability,
   merge boundaries and the risk of modifying primvars, metadata and
   instancing structure. DSIQ initially profiles reuse instead of silently
   rewriting workcell assets.
4. **OpenUSD official performance guidance and NVIDIA instancing guidance**
   distinguish composition/prim-count costs and constraints on editing
   instance proxies. A repeated raw mesh payload does not prove that an
   assembly can be safely converted to a shared prototype.
   https://openusd.org/dev/maxperf.html
   https://docs.omniverse.nvidia.com/usd/latest/learn-openusd/independent/modularity-guide/instancing.html
5. **NIST TN 1297 uncertainty methodology** discusses combining standard
   uncertainties using an RSS calculation under appropriate assumptions.
   DSIQ intentionally does *not* claim a combined uncertainty when it has
   only the physical instrument's stated component.
   https://www.nist.gov/pml/nist-technical-note-1297/nist-tn-1297-5-combined-standard-uncertainty
6. **ROS 2 Jazzy QoS guidance** documents deadline, liveliness and lifespan
   policies. These are future ROS communication integrity test inputs,
   not proof of a safety-rated motion control system.
   https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html
7. **2026 Elias & Issa ITcon operational digital twin study** (user-supplied)
   separates source data, management, semantic context, application logic
   and service delivery. Its case-study result does not establish universal
   manufacturing commissioning or physical machine accuracy.

## Prioritized evidence backlog

- **P0 real equipment:** obtain licensed, source-controlled OEM CAD/URDF,
  inertias, friction/actuator data and calibration records for one workcell.
- **P0 independent physical safety:** obtain workcell-specific risk assessment
  and commissioned safety hardware, guarding and E-stop evidence. No DSIQ
  Python stop latch can replace rated industrial hardware.
- **P0 release integrity:** independently review dependency licenses,
  malicious-update and signing scenarios, and produce a publisher-signed
  Windows release with supported rollback and incident response.
- **P1 RTX 3050:** measure exact FPS/VRAM/RAM, scene load, p95/p99 latency,
  crashes and OOM handling on the target laptop with source-realistic assets.
- **P1 ROS 2 / sensors:** run permissioned hardware-in-the-loop controller,
  TF, QoS, time sync and calibrated sensor trials behind the physical
  safety boundary.
- **P1 learning and users:** perform seeded held-out policy benchmark and
  independently reviewed customer pilot; report failures and workcell
  limitations, not just successes.

Tracking: https://github.com/Niroop-H/DSIQ-Native/issues

## Practical competitive goal

Seek independently measured superiority in a *bounded* task on matching
input assets and hardware: faster offline setup, lower RAM/VRAM, reliable
OpenUSD authoring/round-trip and more transparent source evidence.
General superiority to NVIDIA Omniverse/Isaac Sim/Isaac Lab, full photorealistic
sensor parity or industrial certification is not established by this work.
