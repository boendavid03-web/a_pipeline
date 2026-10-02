# NavIsaacLab source baseline release

This is a **source-only** snapshot of the local 1-human/1-robot reproduction.
It is not a self-contained simulator installation. The original project was
unpacked from a ZIP without `.git`, so there is no verifiable original
NavIsaacLab commit or complete upstream diff. The source code, configurations,
tests, scripts and reports are kept; all acquired assets and generated outputs
remain local and are excluded from Git.

## Included evidence

- [1+1 acceptance report](NAVISAACLAB_1P1R_ACCEPTANCE_20261001.md)
  records the prior bounded runtime acceptance.
- [PPO pilot report](NAVISAACLAB_PPO_BASELINE_1P1R_20261001.md) records the
  bounded 49,920-step training and fixed evaluation; this release did not run
  new PPO experiments.
- [Public baseline manifest](NAVISAACLAB_PPO_BASELINE_PUBLIC_MANIFEST_20261002.yaml)
  retains file and asset hashes with paths relative to this repository or its
  sibling `../Assets/` directory. It is a path-sanitized view of the original
  frozen local manifest, which remains on disk but is not committed.
- `AMASS_SMPL_VALIDATION_MINIMAL_DOWNLOAD.md` documents asset preparation;
  [THIRD_PARTY.md](THIRD_PARTY.md) records the source/asset licensing boundary.

## External asset contract

The configuration refers to `../Assets/motion/amass_smpl_validation.pt`,
`data/pretrained_models/masked_mimic/smpl_57f98a9/last.ckpt`, SMPL files under
`data/smpl/`, and scene/robot USD outside this Git snapshot. These paths must
be populated by the user from authorized sources before running the Isaac
pipeline. The public manifest gives the verified local hashes where available.
The original local evidence under `output/` and sibling `../Assets/repro_logs/`
is not included; a clone alone cannot replay the reported results byte for
byte. The default `env.yaml` requests SMPL mesh rendering, while the reported
pilot used skeleton rendering because the validated environment lacked a
pinned compatible `smplx` installation.

## Reproduction-specific source changes

The local ZIP has no source history, so this is an inventory rather than a
complete diff against an upstream commit. The reproduction added or changed
the 1+1 acceptance and PPO pilot configs, fixed-route evaluator, checkpoint
cadence/seed controls, asset checks, and report-generation scripts. The
original `CrowdSim/config/env.yaml` was not changed for the pilot. Public
release preparation only changed documentation, path handling in diagnostic
scripts, and Git exclusions; it did not change PPO or navigation algorithms.

The diagnostic scripts use paths relative to this checkout where possible.
`scripts/run_fixed_eval_suite.bash` requires `ISAAC_LAB_ROOT` and optionally
accepts `ISAAC_SIM_ROOT`, `ISAACLAB_PYTHON`, `NAVISAACLAB_RUN_DIR` and
`NAVISAACLAB_EVIDENCE_DIR`. It is a historical replay helper and is not run by
the source release process.

The public `README.md` uses text and external project links instead of local
demo media. The original ProtoMotions README copy (`PROTOMOTIONS.md`) and
Sphinx `docs/` tree stay on disk but are excluded from this first commit:
their embedded media references would point to omitted GIF/MP4/LFS assets.
The upstream documentation remains available from the official ProtoMotions
repository linked in `THIRD_PARTY.md`.

## First-commit audit (2026-10-02)

The staged source set contains 317 files (about 3.6 MB). The only binary files
are the two small `CrowdSim/maps/World*.png` occupancy maps. No staged file
exceeds 5 MB; no LFS pointer, credential-shaped string, embedded URL password,
private-key header, or host home-directory path was found by the scoped scan.
All staged relative Markdown links resolve to staged files. This is a pattern
scan of the proposed commit, not a claim about ignored local assets.

`git diff --cached --check` reports existing trailing-space/blank-line issues
in 59 ZIP-sourced files. The release preparation did not bulk-format those
sources, which would change frozen source hashes without changing experiment
behavior. None of the release-preparation files had a whitespace warning.
