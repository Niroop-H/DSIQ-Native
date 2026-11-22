# Industrial asset acquisition and validation

No approved manufacturer-specific CAD geometry ships in this repository. Missing geometry must remain missing—not be replaced by an arbitrary shape labeled "highly detailed".

Each source asset must have a legal provenance document, genuine USD geometry, SHA-256 hash, actual source revision/model, dimensions, axis, assembly part paths, and independently measured/referenced physics data before physical qualification.

Manifest shape: schema=dsiq-asset-v1, asset_id, manufacturer, model, revision, source_uri, license {usage_rights,evidence_file}, geometry {usd_file,sha256,meters_per_unit,up_axis,dimensions_m,subassemblies:[{name,prim_path}]}, physics {qualified:false}. Qualified physical models must additionally have mass_kg and acceptance_report, and undergo independent evidence review.

The validator fails closed on nonexistent CAD, missing rights evidence, invalid hashes and path traversal; it never grants OEM or safety certification. Keep licensed/proprietary CAD in controlled storage outside public Git history.
