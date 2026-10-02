# Third-party source and asset provenance

This repository was prepared from a NavIsaacLab 2.0 ZIP archive without its
original Git metadata. The exact source commit and complete pre-reproduction
diff cannot be reconstructed from that archive. The root [LICENSE](LICENSE)
is the archive's Apache-2.0 license; it does not grant rights to separately
licensed models, motion data, or simulator assets.

## Vendored ProtoMotions source

The `protomotions/` Python source and associated experiment code are derived
from [NVIDIA ProtoMotions](https://github.com/NVlabs/ProtoMotions). Its
Apache-2.0 license from official revision `57f98a9` is retained at
[third_party/ProtoMotions/LICENSE.md](third_party/ProtoMotions/LICENSE.md)
(SHA-256 `59899c6091b540582ed617e8eeaac4919dc985ccfc35459ee9752b699be5205b`).
The local `protomotions/components/motion_lib.py` matches the file at that
revision byte for byte (SHA-256
`4f887ff1b0b81824714c73847b4eacc027711c6a5e39945d43ffb784f1dbfa2e`).
This establishes provenance for that file and a compatible revision, not an
exact commit identity for the entire vendored tree. Existing source copyright
and SPDX headers have been retained. Local reproduction changes are described
in [PUBLIC_RELEASE.md](PUBLIC_RELEASE.md) and the experiment reports.

## Assets outside this Git snapshot

The source release excludes ProtoMotions robot meshes/USD files and LFS
pointers, MaskedMimic weights, SMPL body models, AMASS recordings and derived
MotionLib files, Isaac Sim/Isaac Lab installations, NVIDIA warehouse/robot USD,
example motion data, demo media, and generated logs/checkpoints. Obtain each
asset from its rights holder under its applicable terms. In particular,
ProtoMotions' Apache-2.0 source license does not cover SMPL body models or
AMASS recordings. The included configuration paths and SHA-256 records identify
expected inputs; they do not distribute those inputs.
