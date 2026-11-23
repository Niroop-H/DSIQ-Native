"""Independent exact CAD render geometry fingerprint and safe reuse invariants."""
from dataclasses import replace
import math
import unittest

from dsiq_native.cad_geometry import CADPart
from dsiq_native.render_geometry_cache import render_geometry_fingerprint


class RenderGeometryIdentity(unittest.TestCase):
    def setUp(self):
        self.part=CADPart(
            link_name="frame",visual_name="panel_a",
            vertices=((0.,0.,0.),(1.,0.,0.),(0.,1.,0.)),
            faces=((0,1,2),),rgba=(.4,.5,.6,1.),
            source="original",source_sha256=None,
        )

    def test_reuses_only_exact_render_triangle_buffers(self):
        key=render_geometry_fingerprint(self.part)
        self.assertEqual(key,render_geometry_fingerprint(
            replace(self.part,link_name="other_frame",visual_name="other_panel",
                    source="another_sha_audited_source")
        ))
        for change in (
            {"rgba":(.41,.5,.6,1.)},
            {"faces":((0,2,1),)},
            {"vertices":((0.,0.,0.),(1.00000001,0.,0.),(0.,1.,0.))},
        ):
            with self.subTest(change=change):
                self.assertNotEqual(
                    render_geometry_fingerprint(replace(self.part,**change)),key
                )
        self.assertEqual(len(key),64)

    def test_malformed_geometry_cannot_enter_cache(self):
        for change in (
            {"vertices":((math.nan,0.,0.),(1.,0.,0.),(0.,1.,0.))},
            {"faces":((0,1,3),)},
            {"faces":((0,1,-1),)},
            {"faces":()},
            {"rgba":(.4,.5,.6,math.inf)},
            {"rgba":(.4,.5,2.,1.)},
        ):
            with self.subTest(change=change):
                with self.assertRaises(ValueError):
                    render_geometry_fingerprint(replace(self.part,**change))


if __name__=="__main__":
    unittest.main()
