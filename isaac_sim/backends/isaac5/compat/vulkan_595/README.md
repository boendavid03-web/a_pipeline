# Process-local Vulkan 595 compatibility probe

This is a backend-local copy of the existing Khronos Profiles layer. Its
SHA-256 is checked before a profile is written. The layer is not installed in
system Vulkan directories, does not change the NVIDIA driver, and is enabled
only by `launch/start_isaac_5_1_595_compat.sh`.

Expected layer SHA-256:

`14e5b56e6006ed5fa75536549f87d8c4228d7640d28f4e14eacd460f647d3896`

The previous experiment recorded that the profile probe could override the
reported allocation limit, while a complete Isaac Sim GUI run still suffered
an immediate native crash on this RTX 5090/NVIDIA 595.84 host. This backend
therefore treats the compatibility path as diagnostic and requires fresh
runtime evidence before any stability claim.
