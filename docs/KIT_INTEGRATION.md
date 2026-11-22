# DSIQ Native — optional Kit SDK integration

Use NVIDIA's **official** Kit App Template and a compatible installed Kit Base Editor:
https://github.com/NVIDIA-Omniverse/kit-app-template
https://docs.omniverse.nvidia.com/dev-guide/latest/

This repo contains only DSIQ-authored extension and app descriptors; it does not distribute NVIDIA SDK binaries.

1. Install Kit from an NVIDIA-approved source and accept its current license. Verify the chosen Kit version's driver and hardware requirements.
2. Clone DSIQ-Native. Extension discovery path: source/extensions. DSIQ extension: source/extensions/dsiq.studio.
3. In the officially installed Kit Base Editor, enable the DSIQ extension. Example PowerShell from the SDK directory:

~~~powershell
.\omni.app.editor.base.bat --ext-folder "C:\path\DSIQ-Native\source\extensions" --enable dsiq.studio
~~~

Alternatively, use the built-in explicit low-memory launch guard (from the repository root):

~~~powershell
.\scripts\launch-kit.ps1 -KitExecutable "C:\path\to\omni.app.editor.base.bat" -AllowExperimental
~~~

The guard detects actual GPU VRAM and system RAM before launching. It does not make unsupported NVIDIA Kit configurations officially compatible.

4. Open a real OpenUSD stage through Kit's File menu, then use the DSIQ Native Scene Diagnostics window to inspect the current stage.
5. The file source/apps/dsiq.native.kit is an **experimental Kit app definition** that requires launch/extension validation with the chosen SDK before use or redistribution.

The extension displays real prim and physics-schema counts. It is not yet a substitute for the Kit viewer or Isaac Sim. No native RTX rendering performance measurement has been completed.

**Hardware caveat:** RTX 3050 is below Kit's published RTX 3070 minimum and Isaac Sim's more demanding requirements. If the Kit app does not start on this hardware, use the working headless OpenUSD tooling instead of claiming support.

Licensing: https://docs.omniverse.nvidia.com/dev-overview/latest/common/NVIDIA_Omniverse_License_Agreement.html
