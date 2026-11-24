#!/usr/bin/env bash
set -euo pipefail

manifest="repos/repos-fri-1.17.yaml"
mkdir -p src
# vcs import fetches a repository that is already there, so it runs only when one is missing.
# It skips an empty directory, and a failed clone leaves one behind.
for name in $(sed -n 's/^  \([a-z0-9_]*\):$/\1/p' "${manifest}"); do
    if [[ ! -d "src/${name}/.git" ]]; then
        find src -mindepth 1 -maxdepth 1 -type d -empty -delete
        vcs import src --skip-existing < "${manifest}"
        break
    fi
done

crisp_dir="src/crisp_controllers"
patches=(
    "${PWD}/patches/crisp_controllers-introspection.patch"
    "${PWD}/patches/crisp_controllers-activate-clears-targets.patch"
    "${PWD}/patches/crisp_controllers-rotational-stiffness-ceiling.patch"
    "${PWD}/patches/crisp_controllers-activate-zeroes-torque.patch"
)
# Each patch applies on the one before it, so the last one reversing cleanly means all are on.
if ! git -C "${crisp_dir}" apply --reverse --check "${patches[3]}" 2>/dev/null; then
    cat "${patches[@]}" | git -C "${crisp_dir}" apply
    echo "Applied ${#patches[@]} patches to ${crisp_dir}."
fi

# The UMI description (launch tool:=umi), private to the lab. Its revision fixes the camera
# frame the policy acts in, so a checkout at another one is refused.
iris_dir="src/iris_robots_description"
iris_rev="93c2d99"
if [[ ! -d "${iris_dir}/.git" ]]; then
    if git clone --quiet git@github.com:KAIST-IRiS-Haptics-Telerobotics/iris_robots_description.git \
        "${iris_dir}" 2>/dev/null; then
        git -C "${iris_dir}" checkout --quiet "${iris_rev}"
    else
        echo "No ${iris_dir} (the clone failed): the build goes on, tool:=umi cannot run."
    fi
fi
if [[ -d "${iris_dir}/.git" ]]; then
    actual_iris_rev="$(git -C "${iris_dir}" rev-parse --short=7 HEAD)"
    if [[ "${actual_iris_rev}" != "${iris_rev}" ]]; then
        echo "${iris_dir} is at ${actual_iris_rev}, pinned ${iris_rev}. Check the pin out." >&2
        exit 4
    fi
fi
