# Live handoff, 2026-10-08 02:00 UTC

This Arena stage is summarized in `../STAGE_SUMMARY_20261008.md`; the 18 method ledger is `../METHOD_MATRIX.md`. All 18 have PASS/FAIL/BLOCKED at original quicktest scope: 3/13/2. Nav2 is separate. Preserve dirty worktrees and single-writer/one-Isaac-case ownership.

Current resource state: no Isaac or training compute process; Isaac container exited. ROS domain 189 UDP port 54800 is bound by `/system.slice/todeskd.service` on host IP `[LAN_IP]`. A CADRL candidate launch was rejected before Isaac by preflight, exit 75; case `../RUNS/multimethod_cadrl_headless_20261008T020005Z_2893608` has no navigation result. Do not stop the desktop service merely to run it. Next read-only command: `ss -ulpne | rg '54800|State'`.

The CADRL radius candidate is saved only in `cadrl.radius_candidate.patch` and **not applied** to live source. The live CADRL SHA-256 is original `8ac360879e067949e8d5a3d320c1a9c5933abdcfc6829ffc586726bb1c943b10`. If port is free, apply candidate with exact SHA guard before one bounded run; see `SICNAV_CADRL_DIAGNOSIS.md`. The live CD-SARL goal-stop repair remains SHA-256 `f20cfce687832b0b5139bb82dda6b067c9450380ffb265fd2f3c6b34dce6c225` and requires `A_CDSARL_GOAL_STOP_CANDIDATE=YES` on all guarded runs while present.


The user requested a stage summary now. Pause new Isaac work after handing over this state.
