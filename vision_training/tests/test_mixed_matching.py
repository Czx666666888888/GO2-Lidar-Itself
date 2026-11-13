"""Fixed-threshold diagnostics must preserve class and one-to-one matching."""
import numpy as np
from vision_training.tools.evaluate_science_mixed_v1 import match


def test_class_aware_and_one_to_one():
    gt=np.array([[0,0,0,10,10],[1,20,20,30,30]])
    pred=np.array([[0,0,0,10,10],[0,0,0,10,10],[2,20,20,30,30]])
    gs,ps=match(gt,pred)
    assert gs=={0} and len(ps)==1
    assert match(np.empty((0,5)),pred)==(set(),set())
    assert match(gt,np.empty((0,5)))==(set(),set())
