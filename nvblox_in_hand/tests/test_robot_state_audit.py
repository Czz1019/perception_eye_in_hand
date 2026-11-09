"""Input/pose comparisons for the read-only state audit, no ROS initialization."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
import numpy as np
p=Path(__file__).resolve().parents[1]/'tools/check_simulation.py'
spec=importlib.util.spec_from_file_location('audit',p);audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)
class StateAudit(unittest.TestCase):
    def test_reordered_joint_names_preserve_positions_and_reject_missing_values(self):
        names=list(reversed(audit.JOINTS));msg=SimpleNamespace(name=names,position=list(range(6)))
        q=audit.valid_positions(msg);self.assertEqual(q['wrist_3_joint'],0);self.assertEqual(q['shoulder_pan_joint'],5)
        self.assertIsNone(audit.valid_positions(SimpleNamespace(name=names,position=[0]*5)))
        self.assertIsNone(audit.valid_positions(SimpleNamespace(name=names,position=[float('nan')]*6)))
    def test_detects_transform_translation_and_rotation_errors(self):
        good=audit.pose_residual(np.eye(4),[0,0,0],[0,0,0,1])
        self.assertEqual(good['translation_error_m'],0);self.assertEqual(good['rotation_error_rad'],0)
        bad=audit.pose_residual(np.eye(4),[.1,0,0],[0,0,1,0])
        self.assertAlmostEqual(bad['translation_error_m'],.1);self.assertAlmostEqual(bad['rotation_error_rad'],np.pi)
