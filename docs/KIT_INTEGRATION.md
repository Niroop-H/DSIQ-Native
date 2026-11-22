# DSIQ Native / optional NVIDIA Kit SDK integration

DSIQ's **Kit-hosted studio** now includes an interactive robotics playground
inside an official, separately installed NVIDIA Kit runtime. The repository
contains no proprietary Kit, RTX, or PhysX binaries, and is not a stand-alone
Windows installer.

Official integration: https://github.com/NVIDIA-Omniverse/kit-app-template

## Setup

1. Install a compatible Kit SDK through NVIDIA's official distribution, under
   its current licensing terms. Consult the chosen release's hardware and
   driver requirements.
2. Clone this repository. Its DSIQ extension is
   `source/extensions/dsiq.studio`.
3. Use the Kit app template to include
   `source/apps/dsiq.native.kit` and the local extension search path.
   This manifest requests `omni.physx` to enable a real physics backend,
   in addition to Kit timeline, stage and viewport extensions.
4. Alternatively, enable just the extension inside an installed Kit editor:

```powershell
.scriptslaunch-kit.ps1 -KitExecutable "C:\path\to\omni.app.editor.base.bat" -AllowExperimental
```

This script preflights GPU/RAM and requires an explicit unsupported-hardware
acknowledgment. It cannot make NVIDIA's Kit engine run on an RTX3050
if the SDK requires more RAM or VRAM.

5. Use Kit **File > New** or open a DSIQ-generated USD stage. Open the
   **DSIQ Native | Engineering & Robotics Studio** window.

## Functioning controls

- Pure black (#000000) and light (#FFFFFF) themes for the DSIQ panel. The
  *host Kit viewport* uses its own rendering theme and settings.
- USD inspection, Save (for disk-backed layers), Create/Reset Playground.
- Timeline Play/Pause/Stop using real `omni.timeline` operations. The panel
  reports if `omni.physx` is disabled rather than claiming physics.
- Spawn USD rigid payloads with mass, dimensions, position and colliders.
- Edit six angular joint-drive targets with mechanical limit checks.
- Home robot, position-only Cartesian IK with residual and status.
- Edit physics gravity in the USD scene and inspect live authored schema.

Some host applications do not automatically load the PhysX extension. The
standalone .kit manifest declares the `omni.physx` dependency, but that
does not prove it started or passed a simulation benchmark on your machine.

See `docs/ROBOTICS_PLAYGROUND.md` for exact workflows, tests and limits.

## Supportability

NVIDIA Kit and Isaac Sim's published GPU/RAM minima exceed RTX3050/16 GiB RAM.
This Kit integration is **experimental, unsupported on the specified target**
until measured on real hardware. If it fails, the headless OpenUSD and
kinematics tools are still available.

DSIQ is independent of NVIDIA. For commercial packaging, check current
NVIDIA EULA and redistribution rights of the actual installed SDK release.
