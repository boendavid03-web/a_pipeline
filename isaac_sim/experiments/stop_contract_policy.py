"""Experiment-only zero-command semantics; no production runtime dependency."""
from __future__ import annotations

import math


MODES = ('continuous', 'fixed_stop_target', 'idle_on_stop')


class StopContractPolicy:
    def __init__(self, mode):
        if mode not in MODES:
            raise ValueError(f'unknown stop mode: {mode}')
        self.mode = mode
        self.stopping = False
        self.stop_target = None

    def update(self, speed, requested_target):
        stopped = speed <= 1e-12
        entering = stopped and not self.stopping
        leaving = self.stopping and not stopped
        target = list(requested_target)
        if self.mode == 'fixed_stop_target' and stopped:
            if entering:
                self.stop_target = target
            target = list(self.stop_target)
        action = None
        if self.mode == 'idle_on_stop':
            if entering:
                action = 'idle'
            elif leaving:
                action = 'follow'
        self.stopping = stopped
        return dict(target=target, action=action,
                    force_target_write=self.mode == 'fixed_stop_target' and entering)
