import blenderproc as bproc
import argparse
import json
from pathlib import Path
import sys
import tempfile
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from generate import main

p = argparse.ArgumentParser(description='Check main failure paths with rendering forbidden')
p.add_argument('--config', type=Path, required=True)
p.add_argument('--report', type=Path, required=True)
a = p.parse_args()
c = yaml.safe_load(a.config.read_text())
assets = a.config.resolve().parents[1]/'assets'
c['assets'] = {name:str(assets/filename) for name,filename in c['assets'].items()}
c['scene']['additional_targets'] = [0,0]
c['render']['min_bbox_side_px'] = 10000  # Deliberately impossible, test-only config.
c['render']['max_attempts_per_scene'] = 3
calls = []
def forbidden(*args, **kwargs):
    calls.append('render')
    raise AssertionError('Guard test must never render an image')
bproc.renderer.render = forbidden
bproc.renderer.render_segmap = forbidden
with tempfile.TemporaryDirectory(prefix='science-sampling-guards-') as temp:
    root = Path(temp)
    config = root/'test.yaml'
    config.write_text(yaml.safe_dump(c))
    existing = root/'existing'
    existing.mkdir()
    (existing/'marker').write_text('preserve')
    def run(output):
        sys.argv = ['generate.py','--config',str(config),'--output',str(output),
                    '--per-class','2','--allow-unverified-scale',
                    '--distance-band','far','--image-region','center']
        main()
    try:
        run(existing)
    except FileExistsError:
        pass
    else:
        raise AssertionError('Existing dataset was not refused')
    assert [p.name for p in existing.iterdir()] == ['marker']
    assert (existing/'marker').read_text() == 'preserve'
    output = root/'failed_run'
    try:
        run(output)
    except RuntimeError as error:
        assert 'insufficient valid frames' in str(error), error
    else:
        raise AssertionError('Tiny far targets should fail without completing a dataset')
    manifest = json.loads((output/'manifest.json').read_text())
    rejects = manifest['rejected_attempts']
    assert len(rejects)==3 and manifest['records']==[]
    assert all(r['request']==dict(distance_band='far',image_region='center') for r in rejects)
    assert all(1.5<=r['distance_m']<=2.5 and r['reason']=='projected_target_too_small' for r in rejects)
    assert not (output/'COMPLETE').exists() and (output/'failure.json').exists()
    assert not list(output.rglob('*.png')) and not calls
    a.report.write_text(json.dumps(dict(rgb_images_generated=0, render_calls=0,
        no_overwrite='PASS', pinned_far_request='PASS', no_threshold_relaxation='PASS',
        failure_manifest='PASS', rejected_attempts=rejects),indent=2)+'\n')
print('PASS: no overwrite; pinned far; no fallback; failure manifest; zero render calls')
