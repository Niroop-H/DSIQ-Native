<#
.SYNOPSIS
    Launch DSIQ Native diagnostic extension inside an already installed NVIDIA Kit Base Editor.
.DESCRIPTION
    Uses measured physical RAM and GPU VRAM from the DSIQ CLI before attempting
    to start Kit. An RTX 3050 is below NVIDIA's published Kit minimum. This
    launcher never claims that NVIDIA rendering or PhysX fits the target GPU.
.EXAMPLE
    .\scripts\launch-kit.ps1 -KitExecutable "C:\OmniverseKit\omni.app.editor.base.bat" -AllowExperimental
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$KitExecutable,
    [switch]$AllowExperimental
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$kitPath = (Resolve-Path $KitExecutable -ErrorAction Stop).Path
$extensions = Join-Path $root 'source\extensions'
$cwd = (Get-Location).Path

try {
    Set-Location $root
    $jsonOutput = (& python -m dsiq_native hardware)
    if ($LASTEXITCODE -ne 0) {
        throw 'DSIQ hardware inspection failed. Run python -m dsiq_native hardware.'
    }
    $device = $jsonOutput | ConvertFrom-Json

    Write-Host ('DSIQ GPU: ' + $(if ($null -ne $device.gpu) { $device.gpu.name } else { 'Not detected' }))
    Write-Host ('DSIQ managed-asset advisory budget: ' + $device.managed_asset_budget_mib + ' MiB')
    foreach ($warning in $device.warnings) { Write-Warning $warning }

    if ($device.mode -ne 'experimental_lightweight') {
        throw 'GPU/RAM is not admitted even for a Kit experiment. Headless USD tools remain available.'
    }
    if (-not $AllowExperimental) {
        throw 'Kit on RTX 3050/16 GiB is below NVIDIA published minimums. Re-run with -AllowExperimental to explicitly attempt unsupported hardware.'
    }
    Write-Warning 'Launching unsupported/experimental GPU configuration. Renderer memory use is not capped by this script.'
    & $kitPath --ext-folder $extensions --enable dsiq.studio
    exit $LASTEXITCODE
}
finally {
    Set-Location $cwd
}
