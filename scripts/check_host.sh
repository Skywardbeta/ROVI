#!/usr/bin/env bash
# Read-only host check: reports toolchain versions and locations and exits non-zero when a
# requirement is missing or unusable. It never installs, downloads, uses sudo, or touches the network.
set -euo pipefail

readonly MIN_GCC_VERSION="13"
readonly MIN_CMAKE_VERSION="3.22"
readonly MIN_GTEST_VERSION="1.14"
readonly ROS_PREFIX="/opt/ros/jazzy"
missing=0

report() {
  printf '%-24s %s\n' "$1" "$2"
}

fail() {
  report "$1" "FAIL: $2"
  missing=1
}

version_of() {
  "$@" 2>/dev/null | grep -oE '[0-9]+\.[0-9]+(\.[0-9]+)?' | head -n 1 || true
}

version_at_least() {
  local minimum=$1 actual=$2
  [[ -n ${actual} ]] &&
    [[ $(printf '%s\n%s\n' "${minimum}" "${actual}" | sort -V | head -n 1) == "${minimum}" ]]
}

os_field() {
  sed -n "s/^$1=//p" /etc/os-release 2>/dev/null | head -n 1 | tr -d '"' || true
}

require_tool() {
  local tool=$1 minimum=${2:-} location version
  location=$(command -v "${tool}" 2>/dev/null || true)
  if [[ -z ${location} ]]; then
    fail "${tool}" "not found"
    return
  fi
  version=$(version_of "${tool}" --version)
  if [[ -z ${version} ]]; then
    fail "${tool}" "--version reported no version (${location})"
  elif [[ -n ${minimum} ]] && ! version_at_least "${minimum}" "${version}"; then
    fail "${tool}" "need >= ${minimum}, found ${version} (${location})"
  else
    report "${tool}" "${version} (${location})"
  fi
}

require_command() {
  local tool=$1 location
  location=$(command -v "${tool}" 2>/dev/null || true)
  if [[ -z ${location} ]]; then
    fail "${tool}" "not found"
  else
    report "${tool}" "present (${location})"
  fi
}

optional_tool() {
  local tool=$1 purpose=$2 location
  location=$(command -v "${tool}" 2>/dev/null || true)
  if [[ -z ${location} ]]; then
    report "${tool}" "not installed (optional: ${purpose})"
  else
    report "${tool}" "$(version_of "${tool}" --version) (${location})"
  fi
}

require_path() {
  local label=$1 path=$2
  if [[ -e ${path} ]]; then
    report "${label}" "${path}"
  else
    fail "${label}" "missing ${path}"
  fi
}

optional_ros_package() {
  local package=$1 purpose=$2
  if [[ -d "${ROS_PREFIX}/share/${package}" ]]; then
    report "${package}" "present"
  else
    report "${package}" "not installed (needed for ${purpose})"
  fi
}

check_gtest() {
  local multiarch config version
  multiarch=$(g++ -print-multiarch 2>/dev/null || true)
  if [[ -z ${multiarch} ]]; then
    fail "gtest" "cannot determine the multiarch triplet from g++"
    return
  fi
  config="/usr/lib/${multiarch}/cmake/GTest/GTestConfigVersion.cmake"
  if [[ ! -f ${config} ]]; then
    fail "gtest" "missing ${config}"
    return
  fi
  version=$(sed -n 's/^set(PACKAGE_VERSION "\([0-9.]*\)")$/\1/p' "${config}" | head -n 1 || true)
  if version_at_least "${MIN_GTEST_VERSION}" "${version}"; then
    report "gtest" "${version} (${config%/*})"
  else
    fail "gtest" "need >= ${MIN_GTEST_VERSION}, found ${version:-unknown}"
  fi
}

check_sanitizer_runtime() {
  local library=$1 path
  path=$(g++ -print-file-name="${library}" 2>/dev/null || true)
  if [[ ${path} == /* && -e ${path} ]]; then
    report "${library}" "${path}"
  else
    fail "${library}" "runtime not found by g++"
  fi
}

check_path_entries() {
  local entry
  if [[ ":${PATH}:" == *"::"* ]]; then
    report "PATH" "WARNING: empty entry resolves to the current directory"
  fi
  IFS=':' read -r -a path_entries <<<"${PATH}"
  for entry in "${path_entries[@]}"; do
    if [[ -n ${entry} && ${entry} != /* ]]; then
      report "PATH" "WARNING: relative entry '${entry}'"
    fi
  done
}

report "os" "$(os_field PRETTY_NAME) (ubuntu base: $(os_field UBUNTU_CODENAME))"
report "kernel" "$(uname -r) $(uname -m)"
check_path_entries

require_tool g++ "${MIN_GCC_VERSION}"
require_tool cmake "${MIN_CMAKE_VERSION}"
require_tool make
require_tool git
require_command colcon
require_tool clang-format
require_tool clang-tidy
require_tool cppcheck
require_tool gcov
require_command setarch
require_path "ros jazzy" "${ROS_PREFIX}/setup.bash"
check_gtest
check_sanitizer_runtime libasan.so
check_sanitizer_runtime libubsan.so
check_sanitizer_runtime libtsan.so
report "gcc install dir" "$(dirname "$(g++ -print-libgcc-file-name 2>/dev/null || true)") (clang-tidy --gcc-install-dir)"

format_major=$(version_of clang-format --version | cut -d. -f1)
tidy_major=$(version_of clang-tidy --version | cut -d. -f1)
if [[ -n ${format_major} && ${format_major} != "${tidy_major}" ]]; then
  report "clang tool skew" "WARNING: clang-format ${format_major} vs clang-tidy ${tidy_major}"
fi

optional_tool ninja "faster builds"
optional_tool gcovr "coverage reports"
optional_tool shellcheck "script lint gate"
optional_ros_package rosbag2_storage_mcap "MCAP recording"
optional_ros_package rmw_fastrtps_cpp "the ROS middleware"
optional_ros_package foxglove_bridge "the live dashboard"
optional_ros_package ros_gz_sim "Gazebo scenarios"
optional_ros_package ros_gz_bridge "Gazebo scenarios"

if ((missing)); then
  echo "preflight: FAILED (required items missing or unusable)"
  exit 1
fi
echo "preflight: OK"
