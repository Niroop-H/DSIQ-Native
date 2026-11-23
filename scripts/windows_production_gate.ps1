param(
    [Parameter(Mandatory=$true)][string]$Executable,
    [string]$ExpectedSignerThumbprint = "",
    [string]$GPUPerformanceEvidence = "",
    [string]$MechanicalQualificationEvidence = "",
    [string]$LegalLicenseReview = "",
    [string]$SecurityAuditEvidence = "",
    [string]$IndependentPilotEvidence = ""
)

# DSIQ Native's production evidence gate is intentionally conservative.
# An unsigned PyInstaller preview must NEVER become an asserted
# manufacturer-calibrated, signed industrial simulator.
$ErrorActionPreference = "Stop"
$blocks = [System.Collections.Generic.List[string]]::new()

if (-not (Test-Path -LiteralPath $Executable -PathType Leaf)) {
    $blocks.Add("Executable missing")
} else {
    $signature = Get-AuthenticodeSignature -LiteralPath $Executable
    if ($signature.Status -ne "Valid" -or $null -eq $signature.SignerCertificate) {
        $blocks.Add("Windows Authenticode signature is not cryptographically Valid")
    } elseif ([string]::IsNullOrWhiteSpace($ExpectedSignerThumbprint)) {
        $blocks.Add("Expected publisher certificate thumbprint was not supplied")
    } elseif ($signature.SignerCertificate.Thumbprint -ne ($ExpectedSignerThumbprint -replace '\s', '').ToUpperInvariant()) {
        $blocks.Add("Executable signer does not match independently trusted publisher thumbprint")
    }
}

$requiredEvidence = @{
    GPUPerformanceEvidence = $GPUPerformanceEvidence
    MechanicalQualificationEvidence = $MechanicalQualificationEvidence
    LegalLicenseReview = $LegalLicenseReview
    SecurityAuditEvidence = $SecurityAuditEvidence
    IndependentPilotEvidence = $IndependentPilotEvidence
}
foreach ($key in $requiredEvidence.Keys) {
    $path = $requiredEvidence[$key]
    if ([string]::IsNullOrWhiteSpace($path) -or -not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $blocks.Add("$key required and not supplied")
    }
}

if (-not [string]::IsNullOrWhiteSpace($GPUPerformanceEvidence) -and (Test-Path -LiteralPath $GPUPerformanceEvidence -PathType Leaf)) {
    try {
        $test = Get-Content -LiteralPath $GPUPerformanceEvidence -Raw | ConvertFrom-Json
        if ($test.renderer -ne "Panda3D_OpenGL_rasterization") {
            $blocks.Add("GPU performance report does not contain real OpenGL renderer benchmark")
        }
        if ($test.triangles_in_gpu_scene -lt 100000) {
            $blocks.Add("GPU qualification lacks a complex representative CAD load")
        }
        if ($test.observed_fps -lt 30) {
            $blocks.Add("Measured 30 FPS interactive baseline not reached")
        }
        if ($test.nvidia_rtx_hardware_qualified -ne $true) {
            $blocks.Add("RTX hardware qualification remains unavailable or unverified")
        }
    } catch {
        $blocks.Add("GPU report could not be parsed/validated")
    }
}
$summary = @{
    schema = "dsiq-windows-production-gate-v1"
    result = if ($blocks.Count -eq 0) { "PENDING_INDEPENDENT_HUMAN_REVIEW" } else { "BLOCKED" }
    blockers = @($blocks.ToArray())
    executable = $Executable
    release_certified = $false
    automatic_industrial_qualification = $false
    note = "Even complete files and signatures require independent calibration, source rights, safety and publisher review. This tool cannot grant industrial certification."
}
$summary | ConvertTo-Json -Depth 4
if ($blocks.Count -gt 0) { exit 3 }
exit 0
