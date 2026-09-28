from pathlib import Path
import hashlib


ROOT = Path(__file__).resolve().parents[1]
MODELS = [
    ROOT / 'checkpoints/s3net/s3net_native_stats_best_dev.pth',
    ROOT / 'checkpoints/semantic_cnn/semantic_cnn_native_cmd_best_dev.pth',
    ROOT / 'checkpoints/dr_spaam/ckpt_jrdb_ann_ft_dr_spaam_e20.pth',
    ROOT / 'checkpoints/drl_vo/base_bc_best.pt',
]


def test_required_models_exist_and_are_nonempty():
    assert all(path.is_file() and path.stat().st_size > 0 for path in MODELS)


def test_manifest_hashes_match():
    expected = {
        path: digest for digest, path in
        (line.split(maxsplit=1) for line in (ROOT / 'SHA256SUMS').read_text().splitlines() if line)
    }
    for path in MODELS:
        name = str(path.relative_to(ROOT))
        assert expected[name] == hashlib.sha256(path.read_bytes()).hexdigest()


def test_manifest_covers_every_deployable_file():
    listed = {
        path for _, path in
        (line.split(maxsplit=1) for line in (ROOT / 'SHA256SUMS').read_text().splitlines() if line)
    }
    actual = {
        str(path.relative_to(ROOT))
        for path in ROOT.rglob('*')
        if path.is_file()
        and path.name != 'SHA256SUMS'
        and '__pycache__' not in path.parts
        and path.suffix not in ('.pyc', '.pyo')
    }
    assert listed == actual
