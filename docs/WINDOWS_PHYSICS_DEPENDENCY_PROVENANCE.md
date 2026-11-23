# Windows PyBullet distribution — release provenance gate

**Checked:** 23 November 2026, against the public PyPI release pages.

DSIQ currently pins `pybullet==3.2.7` on non-Windows and
`pybullet-arm64==3.2.8` on Windows in `pyproject.toml`.
**Do not infer that the latter is the Bullet project's official Windows
distribution from its module import name or a green CI result.**

The `pybullet-arm64` PyPI project describes itself as a maintained fork
originally providing macOS arm64 build fixes and a drop-in PyBullet
replacement. Release 3.2.8 includes files tagged for Windows
`win_amd64` as well as macOS/Linux. In particular, the CPython 3.11
Windows x64 wheel is published independently from the `pybullet`
project. Its PyPI distribution listing did not present PyPI provenance
attestations or Trusted Publishing for the observed release.

Sources:
- https://pypi.org/project/pybullet-arm64/
- https://pypi.org/project/pybullet/
- https://github.com/bulletphysics/bullet3/issues/4789
- https://www.piwheels.org/project/pybullet-arm64/

**Current interpretation:** CI exercising the Windows wheel proves a
bounded runtime contract on that runner. It does NOT independently
establish the legal right to redistribute binaries, publisher identity,
reviewed native build flags, equivalence to Bullet 3.2.7, or the absence
of malicious changes in compiled native code.

## Mandatory commercial release review

1. Record exact wheel filename, immutable SHA-256, source distribution,
   release origin, maintainers, repository and reproducible build
   instructions for each platform.
2. Compare native extension interfaces and release history against
   official Bullet source; inspect bundled DLLs, licences, extra
   dependencies and transitive notices.
3. Check the wheel with installed security and malware scanning tools,
   document any missing attestations, and keep an approved package
   allowlist. Audit `pybullet-arm64` independently of a Python-only
   dependency scan.
4. Confirm whether redistributing the wheel in standalone Windows
   PyInstaller/.exe/.msi installers is authorised and whether notices
   and source availability obligations apply.
5. Recheck the Windows CPU and GUI smoke tests with the approved exact
   binary build and physical host. Never waive the safety disclaimer.
6. Prefer source-verifiable/maintainer-supported alternate builds if the
   required provenance evidence is unavailable.

**Gate status:** OPEN / NOT APPROVED FOR COMMERCIAL BINARY RELEASE.
No Python dependency version was changed merely based on the package
name or an incomplete public metadata review.
