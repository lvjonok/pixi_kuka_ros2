# Choosing the FRI client version

The FRI client SDK must **exactly match** the FRI version installed in Sunrise.OS.
Read the installed version from the smartPAD before building anything.

## Supported versions

LBR-Stack ships manifests for `1.11`, `1.14`, `1.15`, `1.16`, `2.5`, `2.6`, and `2.7`
on ROS 2 Jazzy. It does **not** ship one for `1.17`, so this workspace adds
`repos/repos-fri-1.17.yaml`, which mirrors upstream's 1.16 manifest against the
`fri-1.17` branch of `lbr-stack/fri`.

`scripts/import_sources.sh` prefers a local `repos/repos-fri-<version>.yaml` over the
upstream manifest, so adding a version means adding a manifest here.

## Selecting one

The workspace defaults to **FRI 1.17**. For another version:

1. pass it while importing sources, for example `FRI_CLIENT_VERSION=1.16 pixi run -e jazzy setup`; and
2. set the matching `major_version` and `minor_version` in
   `config/lbr_system_config.yaml` before building.

## How mismatches surface

Only the **major** version is enforced at runtime: `lbr_ros2_control` compares it
against the compiled SDK and refuses to start on a mismatch.

The **minor** version is parsed but never checked. An incorrect minor number therefore
fails silently at the FRI handshake instead of producing a clear error, which is the
harder failure to diagnose. Get it right rather than relying on the check.

## One version per workspace

Do not mix source packages from two FRI versions in the same `src` directory.
`import_sources.sh` refuses to proceed if `src/fri` is already checked out on a branch
other than the requested one, and the fix is a fresh `src` directory rather than a
branch switch in place.
