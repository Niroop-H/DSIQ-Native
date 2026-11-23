# DSIQ Native — 100-Point Industrial Qualification Plan

**Basis:** DSIQ Native v0.2.0, commit 16d1305, inspected 23 November 2026.
The prior 42/100 was a weighted engineering judgment, NOT an ISO
certificate, numerical fidelity measurement or external expert score.

**Goal:** pass the complete acceptance test set for a specifically
declared industrial use case. A passing software test, SHA-256 hash or
self-authored "evidence" file NEVER constitutes OEM calibration,
functional-safety approval, customer acceptance or a physical E-stop.

## 100-point assessment — baseline unchanged until independently reviewed

| Area | Baseline | Maximum | What earns the remainder |
|---|---:|---:|---|
| Physics & robotics | 10 | 20 | Continuously checked motion, torque/acceleration models, measured physical validation |
| CAD & OpenUSD | 8 | 15 | Licensed calibrated models, large assembly workflows and sourced material fidelity |
| Native GPU editor | 8 | 15 | Qualified RTX 3050 tests, editing constraints, scalable geometry and stability |
| ROS 2 & sensors | 6 | 10 | Hardware controller, TF, calibrated sensor and safety-interface commissioning |
| Physical AI | 3 | 10 | Independent evaluations, domain-randomized learning and demonstrated transfer |
| Windows distribution | 5 | 10 | Signed installer/store packaging, rollback, compatibility and customer support |
| Security, licensing, updates | 2 | 10 | Full vulnerability/dependency assessment, lawful rights and protected updates |
| Independent industrial validation | 0 | 10 | Real equipment measurements, risk evaluation and external accepted pilot |
| **TOTAL** | **42** | **100** | **Independent evidence required** |

## Eight hard P0/P1 conditions

Every launch gate requires real files, their original SHA-256 digests and
separate independent engineering review. The new launch-evidence-audit
command verifies file integrity and reports missing items; it
intentionally never approves industrial release or awards points.

| Category | Priority | Evidence demanded |
|---|---|---|
| P0_ROBOT_SAFETY | P0 | Machine risk assessment; ISO 10218-1/2:2025 review; safety hardware, guarding, E-stop, collision and failure testing |
| P0_MACHINE_CALIBRATION | P0 | OEM CAD usage rights, measured mass, inertia, friction and actuator limits; instrumented tolerances and uncertainty |
| P0_RELEASE_SECURITY | P0 | Trusted signing, secure update/rollback, approved licenses, scanned binary SBOM, remediation log |
| P1_RTX3050_PERFORMANCE | P1 | Real RTX3050 / 16GB RAM representative load, FPS, peak VRAM, OOM recovery and sustained-run stability |
| P1_EDITOR_WORKFLOWS | P1 | Mechanical mates/constraints, hierarchy editing, save/reopen, real object bounds, undo and scene scaling tests |
| P1_ROS2_HARDWARE | P1 | Real commissioned controllers/TF/sensors with independently tested functional-safety system |
| P1_PHYSICAL_AI | P1 | Seeded training, held-out benchmarks, domain shift tests and physically measured transfer errors |
| P1_CUSTOMER_ACCEPTANCE | P1 | Controlled customer pilot, defect log, documented corrections and independent sign-off |

### Example launch evidence file

The following is a SCHEMA EXAMPLE ONLY; its checksum and source are
intentionally not legitimate qualification evidence:

    {
      "schema":"dsiq-launch-evidence-inventory-v1",
      "evidence":{
        "P0_ROBOT_SAFETY":{
          "attachments":[
            {"file":"real_risk_assessment.pdf",
             "sha256":"REPLACE_WITH_REAL_SHA256_HEX"}
          ]
        }
      }
    }

    python -m dsiq_native launch-evidence-audit evidence/launch.json

The audit is fail closed. A verified digest ONLY proves the file has
not changed, not that its content is true or its author is qualified.

## Motion analytics + simulation stop logic

The new path control accepts an explicitly authored document of
source-URDF joint speed/acceleration/jerk limits, checks the exact
analytical quintic motion extrema and refuses an out-of-envelope move.

    python -m dsiq_native robot-urdf-motion machine.manifest.json --target actuator_x=0.25 --motion-limit-file reviewed-motion-limits.json --require-reviewed-limits

Required limits-file schema:

    {
      "schema":"dsiq-document-controlled-joint-motion-limits-v1",
      "asset_id":"SOURCE_ASSET_ID",
      "joints":{
        "actuator_x":{"joint_type":"prismatic",
          "max_speed":0.3,"max_acceleration":0.5,"max_jerk":1.0}
      }
    }

Include **every actuated axis**; adapt only to real source limits.
The software stop latch is NOT a physical emergency-stop function.
The existing path checker remains sampled rather than continuous.
Proper physical safety requires independent rated safety control,
validation, and model-specific risk assessment.

## Secure offline update handling

Install the optional signature-verification dependency:

    python -m pip install -e ".[security]"

Verify a detached Ed25519 signature against an **externally trusted,
independently pinned public key**, the complete archive SHA-256 and
strict monotonic version progression:

    python -m dsiq_native update-verify update.json update.sig trusted-publisher.pub DSIQ-Native-1.2.3.zip --installed-version 1.2.2

No signing keys are generated or included in release artifacts.
Verification does not install updates or establish the publisher's
real-world identity; Windows Authenticode/Store signing remains a
separate gate.

Microsoft's current docs distinguish MSIX packages signed by the
Microsoft Store on submission from EXE/MSI binaries not re-signed
automatically by that distribution path.

## Measured physical-AI diversity baseline

    python -m dsiq_native domain-randomization --envs 4 --iterations 24 --seed 1007

This actually changes per-client Bullet gravity/contact friction,
then measures robot feedback and simulated dynamics. It is a
bounded CPU experiment, not GPU Isaac Lab, validated RL training or
evidence of physical transfer. Domain randomization becomes useful
when its parameter ranges are grounded in manufacturer measurements.

## Target workstation benchmark

    python -m dsiq_native gpu-benchmark --frames 120 --width 1280 --height 720 --min-fps 30 --require-nvidia --manifest real-machine.manifest.json > RTX3050-evidence.json

A short benchmark is only a screening test. Full acceptance requires
the operator's actual RTX 3050 and 16GB RAM, approved large CAD,
versioned driver and device identity, peak VRAM and process RAM,
physics/sensor error and long-duration memory/latency checks.
Do not infer NVIDIA performance from CI's Mesa OpenGL renderer.
The 30 FPS target is an engineering proposal, not an OEM standard.

## External work that cannot be completed solely in software

1. Obtain legally usable CAD, mass/inertia data, actuator specs and
   instrumented calibration records from a real machine or OEM.
2. Engage a competent robot-cell safety assessor and commission
   independent physical guarding, emergency stop and control tests.
3. Run DSIQ on the intended RTX3050/16GB workstation with
   representative real scenes and publish full results.
4. Have a trusted publisher produce Windows-signed releases and
   approved software-license and security assessments.
5. Arrange a controlled customer pilot; record defects and obtain
   independent acceptance signatures for the bounded workcell scope.

## Definition of 100/100

100/100 for a specific commercial scope means all weighted categories
have independently reviewed *real* evidence, acceptable residuals
under machine-specific tolerances, a safe deployment boundary,
security/rights clearance and customer acceptance. It does NOT
mean perfect numerical physics, all industrial equipment supported,
Omniverse parity or universal safety certification.

## Authoritative scope and security sources

- ISO 10218-1:2025 industrial robot safety summary:
  https://www.iso.org/standard/73933.html
- ISO 10218-2:2025 robot cells and integration:
  https://www.iso.org/standard/73934.html
- NIST SP 800-218 SSDF v1.1:
  https://csrc.nist.gov/pubs/sp/800/218/final
- Microsoft MSIX/store signing:
  https://learn.microsoft.com/en-us/windows/msix/package/sign-msix-package-guide

The supplied references are bibliographic pointers, not claims that
DSIQ has complied with or been certified against those documents.
