"""Verify read-only hooks count called steps and record actual loader schedule."""
import json
from types import SimpleNamespace
from vision_training.runtime_audit import attach_runtime_audit


def test_runtime_audit_records_schedule_steps_and_cleans_hook(tmp_path):
    callbacks={}
    model=SimpleNamespace(add_callback=lambda name,fn:callbacks.update({name:fn}))
    handles=[]
    def register(fn):
        handles.append(fn)
        return SimpleNamespace(remove=lambda:handles.clear())
    trainer=SimpleNamespace(batch_size=8,train_loader=list(range(75)),
        args=SimpleNamespace(nbs=64,warmup_epochs=3),accumulate=8,
        optimizer=SimpleNamespace(register_step_post_hook=register),epoch=0,
        scaler=SimpleNamespace(get_scale=lambda:65536))
    attach_runtime_audit(model,tmp_path)
    callbacks['on_pretrain_routine_end'](trainer)
    handles[0](None,None,None)
    callbacks['on_train_batch_end'](trainer)
    setup=json.loads((tmp_path/'runtime_setup.json').read_text())
    assert setup['warmup_batch_boundary']==225
    assert setup['initial_accumulate']==8
    record=json.loads((tmp_path/'batch_audit.jsonl').read_text())
    assert record['optimizer_steps']==record['batches']==1
    callbacks['on_train_end'](trainer)
    assert not handles
    assert json.loads((tmp_path/'runtime_result.json').read_text())==dict(batches=1,optimizer_steps=1)
