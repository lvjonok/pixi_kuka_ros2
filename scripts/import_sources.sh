#!/usr/bin/env bash
set -euo pipefail

fri_client_version="${FRI_CLIENT_VERSION:-1.17}"
stack_dir="src/lbr_fri_ros2_stack"
crisp_dir="src/crisp_controllers"
crisp_py_dir="external/crisp_py"

mkdir -p src

if [[ ! -d "${stack_dir}/.git" ]]; then
    git clone --depth 1 --branch jazzy \
        https://github.com/lbr-stack/lbr_fri_ros2_stack.git "${stack_dir}"
fi

# A local manifest takes precedence, so this workspace can support an FRI
# version that lbr_fri_ros2_stack does not ship on jazzy (currently 1.17).
local_manifest="repos/repos-fri-${fri_client_version}.yaml"
stack_manifest="${stack_dir}/lbr_fri_ros2_stack/repos-fri-${fri_client_version}.yaml"
if [[ -f "${local_manifest}" ]]; then
    manifest="${local_manifest}"
elif [[ -f "${stack_manifest}" ]]; then
    manifest="${stack_manifest}"
else
    echo "Unsupported FRI client version: ${fri_client_version}" >&2
    echo "Upstream ships: 1.11, 1.14, 1.15, 1.16, 2.5, 2.6, 2.7" >&2
    echo "This workspace additionally ships: 1.17" >&2
    exit 2
fi

if [[ -d "src/fri/.git" ]]; then
    actual_fri_branch="$(git -C src/fri branch --show-current)"
    expected_fri_branch="fri-${fri_client_version}"
    if [[ "${actual_fri_branch}" != "${expected_fri_branch}" ]]; then
        echo "src/fri is on ${actual_fri_branch}, but ${expected_fri_branch} was requested." >&2
        echo "Use a fresh src directory when changing FRI client versions." >&2
        exit 3
    fi
fi

required_repositories=(
    fri
    lbr_fri_idl
    lbr_iiwa7_r800_description
    lbr_iiwa14_r820_description
    lbr_med7_r800_description
    lbr_med14_r820_description
)
all_repositories_present=true
for repository in "${required_repositories[@]}"; do
    if [[ ! -d "src/${repository}/.git" ]]; then
        all_repositories_present=false
        break
    fi
done

if [[ "${all_repositories_present}" == "false" ]]; then
    vcs import src --skip-existing < "${manifest}"
fi

if [[ ! -d "${crisp_dir}/.git" ]]; then
    git clone --depth 1 https://github.com/learnsyslab/crisp_controllers.git "${crisp_dir}"
fi

# crisp_py is the Python client used to drive the arm from scripts. It is also
# installed as the crisp-python pypi dependency; the clone is here for its
# examples and for reading the source. COLCON_IGNORE keeps colcon out of it.
if [[ ! -d "${crisp_py_dir}/.git" ]]; then
    mkdir -p "$(dirname "${crisp_py_dir}")"
    git clone --depth 1 https://github.com/learnsyslab/crisp_py.git "${crisp_py_dir}"
fi
touch "${crisp_py_dir}/COLCON_IGNORE"

# The arm WITH what is bolted to it: iiwa14 + UMI backplate + D405 + tweezer jaws, so the
# controllers can be pointed at the camera (launch tool:=umi). Private to the lab org, and
# pinned by commit because the frame the policy is driven in is defined by this file -- a moved
# branch would move the camera frame under a running experiment with nothing to show for it.
# Upstream lbr_ros2_control's iiwa14.xacro stays the default description (tool:=none).
iris_dir="src/iris_robots_description"
iris_rev="${IRIS_DESCRIPTION_REV:-9eef1e6}"
if [[ ! -d "${iris_dir}/.git" ]]; then
    git clone git@github.com:KAIST-IRiS-Haptics-Telerobotics/iris_robots_description.git "${iris_dir}"
    git -C "${iris_dir}" checkout --quiet "${iris_rev}"
fi
actual_iris_rev="$(git -C "${iris_dir}" rev-parse --short=7 HEAD)"
if [[ "${actual_iris_rev}" != "${iris_rev:0:7}" ]]; then
    echo "${iris_dir} is at ${actual_iris_rev}, pinned ${iris_rev}. Check it out, or set" >&2
    echo "IRIS_DESCRIPTION_REV to say the move is deliberate." >&2
    exit 4
fi

echo "Sources are present for FRI ${fri_client_version} (manifest: ${manifest})."
