"""Read-only public training hooks for loader, warmup and optimizer step evidence."""
import json
from pathlib import Path
try:
    from .common import sha256, write_json
except ImportError:
    from common import sha256, write_json


def attach_runtime_audit(model, output):
    output = Path(output)
    state = dict(batches=0, optimizer_steps=0)
    def stepped(optimizer, args, kwargs):
        state['optimizer_steps'] += 1
    def setup(trainer):
        import ultralytics
        source = Path(ultralytics.__file__).parent/'engine/trainer.py'
        nb = len(trainer.train_loader)
        write_json(output/'runtime_setup.json', dict(
            batch=trainer.batch_size, nbs=trainer.args.nbs, batches_per_epoch=nb,
            warmup_epochs=trainer.args.warmup_epochs,
            warmup_batch_boundary=max(round(trainer.args.warmup_epochs*nb),100),
            initial_accumulate=trainer.accumulate,optimizer=type(trainer.optimizer).__name__,
            trainer_sha256=sha256(source),ultralytics=ultralytics.__version__))
        state['handle'] = trainer.optimizer.register_step_post_hook(stepped)
    def batch_end(trainer):
        state['batches'] += 1
        with (output/'batch_audit.jsonl').open('a') as f:
            f.write(json.dumps(dict(epoch=trainer.epoch+1,batches=state['batches'],
                optimizer_steps=state['optimizer_steps'],accumulate=trainer.accumulate,
                amp_scale=float(trainer.scaler.get_scale())))+'\n')
    def end(trainer):
        state.pop('handle').remove()
        write_json(output/'runtime_result.json',state)
    model.add_callback('on_pretrain_routine_end',setup)
    model.add_callback('on_train_batch_end',batch_end)
    model.add_callback('on_train_end',end)
