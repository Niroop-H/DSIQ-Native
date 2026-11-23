"""SPDX 2.3 installed-component inventory for unsigned DSIQ previews.

Generates a machine-readable inventory of the build environment's
installed DSIQ/physics/CAD/ROS renderer Python distributions. It
does not establish a reproducible build, vulnerability absence,
license rights, or a signed software supply chain.
"""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
from pathlib import Path
import re

COMPONENTS=("dsiq-native","numpy","pybullet","pybullet-arm64",
            "panda3d","trimesh","usd-core","Pillow","PyInstaller")


def build_spdx_inventory(file: str | Path) -> dict:
    target=Path(file).resolve()
    if target.exists():
        raise FileExistsError("SBOM file exists; refusing overwrite")
    packages=[]
    for name in COMPONENTS:
        try:
            dist=metadata.distribution(name)
        except metadata.PackageNotFoundError:
            continue
        actual=dist.metadata.get("Name",name)
        version=dist.version
        if not re.fullmatch(r"[A-Za-z0-9_.+-]+",version):
            raise ValueError("Package version incompatible with constrained SPDX generator")
        lic=(dist.metadata.get("License-Expression") or
             dist.metadata.get("License") or "NOASSERTION").strip()
        # Package metadata can be free text; unsupported SPDX expressions
        # are kept as commentary, never claimed a verified legal license.
        normalized=lic if re.fullmatch(r"[A-Za-z0-9.+_-]+",lic) else "NOASSERTION"
        pid="SPDXRef-Package-"+re.sub(r"[^A-Za-z0-9.-]","-",actual)
        packages.append({
            "SPDXID":pid,
            "name":actual,
            "versionInfo":version,
            "downloadLocation":"NOASSERTION",
            "filesAnalyzed":False,
            "licenseConcluded":"NOASSERTION",
            "licenseDeclared":normalized,
            "copyrightText":"NOASSERTION",
            "comment":"Only installed distribution metadata; do not infer distribution rights",
        })
    packages.sort(key=lambda p:p["name"].lower())
    if not any(p["name"].lower()=="dsiq-native" for p in packages):
        raise ValueError("Build environment does not contain installed DSIQ Native distribution")
    digest=hashlib.sha256(json.dumps(packages,sort_keys=True).encode()).hexdigest()
    document={
        "spdxVersion":"SPDX-2.3",
        "dataLicense":"CC0-1.0",
        "SPDXID":"SPDXRef-DOCUMENT",
        "name":"DSIQ-Native-installed-python-components",
        "documentNamespace":"https://mahquantum.tech/spdx/dsiq-native/"+digest,
        "creationInfo":{
            "created":datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "creators":["Tool: DSIQ Native installed-component inventory"],
        },
        "packages":packages,
        "documentDescribes":[p["SPDXID"] for p in packages],
        "annotations":[],
        "comment":"Not complete binary SBOM, not independently license-audited, not a software-signing certificate.",
    }
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(document,indent=2)+"\n",encoding="utf8")
    return {
        "path":str(target),
        "schema":"SPDX-2.3",
        "installed_components":len(packages),
        "package_names":[p["name"] for p in packages],
        "license_policy_independently_cleared":False,
        "binary_dependencies_exhaustively_inventoried":False,
        "security_vulnerabilities_audited":False,
    }
