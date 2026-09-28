"""Ensure stop ablations cannot restart a task every control frame."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'experiments'))
from stop_contract_policy import StopContractPolicy


def test_continuous_preserves_existing_mapping_even_at_zero():
    policy = StopContractPolicy('continuous')
    for speed, target in [(1, [3, 4, 0]), (0, [2, 4, 0]), (0, [2.1, 4, 0]), (1, [3.1, 4, 0])]:
        assert policy.update(speed, target) == dict(target=target, action=None, force_target_write=False)


def test_fixed_target_does_not_follow_drifting_root_and_refreshes_next_stop():
    policy = StopContractPolicy('fixed_stop_target')
    assert policy.update(0, [2, 4, 0])['force_target_write']
    assert policy.update(0, [2.1, 4, 0])['target'] == [2, 4, 0]
    assert policy.update(1, [4, 4, 0])['target'] == [4, 4, 0]
    assert policy.update(0, [3, 4, 0])['target'] == [3, 4, 0]


def test_idle_and_follow_only_on_edges():
    policy = StopContractPolicy('idle_on_stop')
    speeds = [1] * 120 + [0] * 120 + [1] * 120
    actions = [(i, policy.update(speed, [2, 4, 0])['action']) for i, speed in enumerate(speeds)]
    assert [(i, a) for i, a in actions if a] == [(120, 'idle'), (240, 'follow')]


def test_unknown_mode_rejected():
    with pytest.raises(ValueError):
        StopContractPolicy('cancel_every_frame')
