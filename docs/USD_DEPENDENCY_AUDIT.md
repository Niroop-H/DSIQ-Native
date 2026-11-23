# DSIQ Native — Read-only OpenUSD dependency audit

**Implemented:** a bounded provenance inventory using the real Pixar `UsdUtils.ComputeAllDependencies` API. This extends the scene's geometry and USD Physics validator: it verifies which **local files** a stage resolves, not whether their geometry is physically accurate.

## Execute

```powershell
python -m pip install -e ".[usd]"
python -m dsiq_native usd-dependency-audit project/root.usda --asset-root project
python -m unittest discover -s tests -p test_usd_dependency_audit.py -v
```

The JSON result includes all discovered and approved layer/texture files, relative paths, SHA-256 hashes, sizes, source types, unresolved references and exact policy limits. Failures are explicit; a missing reference or escaped local path yields `passed: false`, and exceeding an inspection limit raises a bounded exception.

Safety budgets: **256 files**, **256 MiB total**, **96 MiB per file**, **128 errors**. The module checks root stage bytes before and after enumeration and checks each hashed file's size/mtime before and after reading. It does not edit layers or write replacement geometry. Unit tests use actual Pixar USD reference arcs and an authored asset-valued texture dependency.

## Security model and gaps

1. **USD resolver execution is not sandboxed.** `UsdUtils.ComputeAllDependencies` may call external `Ar` resolver plugins *before* DSIQ has approved a returned path. Only inspect trusted or separately sandboxed scenes. The reported allowed-local-root check does **not** stop a custom resolver from making network requests.
2. It is not a replacement for a secure URI resolver, content trust validator, quarantined asset ingestion pipeline or package signature.
   **Budget limits are applied to enumerated files after USD has resolved the graph.** They do not preempt the memory, runtime or network cost of resolver traversal. An untrusted USD file requires an isolated subprocess with its own timeout, resource limits, approved resolver and network policy before this command is suitable for automated ingestion.
3. Archive-internal `.usdz` dependencies and dynamic/runtime-only lookups are **not** checked. Input USDZ fails explicitly.
4. The audit is bounded and only covers dependencies returned by Pixar's graph resolver; it cannot guarantee all dynamic shader or extension runtime assets have been discovered.
5. A file hash establishes a particular byte sequence, **not** an OEM CAD right, sensor calibration, asset physical fidelity or a particular renderer's material compatibility.

## Integration acceptance

The GitHub OpenUSD job creates a real authored USD engineering scene, runs the auditor, checks `passed=true`, verifies that the report has at least the root file, and archives a machine-readable JSON inventory. The regression suite exercises loaded local composition and texture, missing references, an existing file outside the root, unsupported dynamic/remote schemes and strict budgets.

## Upstream API references

- [Pixar OpenUSD dependency enumeration](https://openusd.org/25.11/api/dependencies_8h.html)
- [Pixar USD `SdfLayer` external-asset/reference metadata](https://openusd.org/release/api/class_sdf_layer.html)

This is a foundation for scene packaging, scene streaming and collaboration; it is **not** Hydra/Vulkan rendering, Nvidia SimReady conformance, or OpenUSD remote stage replication.
