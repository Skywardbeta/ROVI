# ROVI

ROVI is a deadline-aware compute and network-service scheduler for edge-connected robots. A robot uploads an observation, an edge worker runs inference, and the resulting action chunk must arrive before the robot's action buffer runs out. ROVI decides whether, when, and where each of these chains runs by reserving uplink service, compute time, and downlink service together.

It is a C++20 research prototype on ROS 2 Jazzy and Gazebo Harmonic.

## What it does

- **Deadline-aware admission.** A job's deadline is the earliest of its action-buffer depletion minus a safety margin, its task deadline, and its result-freshness bound. A chain is admitted only if upload, inference, and download all fit before it.
- **Transactional reservation.** Each cycle orders feasible jobs by least slack and commits each chain, or a whole cooperative group, all-or-none into uplink, downlink, and compute calendars. Committed reservations never move.
- **Cooperative groups.** The three arms of a workcell are admitted together, and their action chunks are released together within a skew bound.
- **Strict execution.** The logical link delivers each payload at its reserved time and refuses late or unauthorized payloads instead of shifting them. A robot without a fresh chunk falls back locally and holds still.
- **Policy comparison.** An offline tool compares least slack, earliest deadline, and first-in-first-out against independent-stage controls on one execution engine, with optional per-robot caps and best-effort recovery.

ROVI schedules an abstract end-to-end network service, not Wi-Fi or 5G radio resources. Network profiles are synthetic inputs, and the scheduler is a heuristic with no optimality or hard real-time guarantee. See [docs/algorithm.md](docs/algorithm.md) and [docs/network-service-model.md](docs/network-service-model.md).

## Repository layout

| Path | Contents |
|---|---|
| `src/rovi_core` | ROS-independent core: strong units, capacity calendars, admission, scheduler, audit, execution engine, and the `rovi_compare` tool |
| `src/rovi_interfaces` | ROS 2 messages |
| `src/rovi_scheduler` | Scheduler component |
| `src/rovi_network` | Logical wireless link component |
| `src/rovi_worker` | Simulated inference worker component |
| `src/rovi_robot_agent` | Robot agent, arm controller, and waypoint controller components |
| `src/rovi_coordination` | Cooperative group coordinator component |
| `src/rovi_telemetry` | Shared run clock and settings publication |
| `src/rovi_sim` | Gazebo worlds, robot models, and bridge configurations |
| `src/rovi_bringup` | Launch files, scenario parameters, and Foxglove layouts |
| `src/rovi_tests` | Integration tests and headless Gazebo smoke tests |
| `scripts` | Host check, model fetch, scenario generation, and run recording |
| `external` | Pinned LeRobot and openpi submodules, not built by colcon |

## Requirements

Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz`), `rosbag2_storage_mcap`, `foxglove_bridge`, GCC 13 or newer, CMake, and colcon. `scripts/check_host.sh` reports anything missing without changing the host.

## Build and test

Run from the repository root. The first command fetches the Franka Panda model (about 55 MB from Gazebo Fuel, checksum-pinned; see `src/rovi_sim/models/THIRD_PARTY.md`).

```bash
scripts/fetch_models.sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test
colcon test-result --verbose
```

The external repositories are optional: `git submodule update --init` fetches them at their pinned commits.

## Run a scenario

| Scenario | What runs |
|---|---|
| `vertical_slice` | One robot, the scheduler, one worker, and the logical link, without a simulator |
| `sim_one_robot` | One mobile robot in a Gazebo warehouse |
| `arm_workcell` | One Panda arm in a pick-and-place workcell |
| `thirty_arms` | Thirty Pandas in ten cooperative workcells sharing one access point and three workers |

The Gazebo scenarios include a synthetic network outage from 6 s to 7.5 s: the robots hold while their buffers are empty and resume afterwards.

Record a headless run to MCAP under `bags/`:

```bash
scripts/run_headless.sh arm_workcell 20
```

The script saves the run manifest and parameters with the bag, and fails if a required topic is missing or ROS time moved backward. Thirty arms run slower than real time, and all timing uses simulation time, so `scripts/run_headless.sh thirty_arms 180` records about 27 simulated seconds.

Watch a scenario live:

```bash
source install/setup.bash
ros2 launch rovi_bringup arm_workcell.launch.xml
```

Gazebo opens (through XWayland on Wayland desktops). For the dashboard, connect Foxglove or Lichtblick to `ws://127.0.0.1:8765` and import `src/rovi_bringup/foxglove/arm_workcell.json` or `sim_one_robot.json`. Add `autonomy:=false` to move the arm by hand with Gazebo's Joint Position Controller plugin. If others run Gazebo on your network, export a private `GZ_PARTITION` first.

The thirty-arm world, bridge, parameters, and launch file are generated: edit `scripts/generate_thirty_arms.sh`, run it, and rebuild.

## Compare scheduling policies

This needs no ROS or Gazebo:

```bash
install/rovi_core/bin/rovi_compare --policies independent_earliest_deadline,linked_least_slack
scripts/run_comparison.sh --install-base install "bags/comparison_$(date -u +%Y%m%dT%H%M%SZ)" --robots 6
```

The first command prints a YAML report. The second also saves it with a provenance manifest. The first listed policy is the reference when runs are paired.

## Development checks

<details>
<summary>Formatting and static analysis</summary>

```bash
PACKAGES="rovi_core rovi_telemetry rovi_scheduler rovi_network rovi_worker rovi_coordination rovi_robot_agent rovi_tests"
find src \( -name '*.hpp' -o -name '*.cpp' \) -print0 | xargs -0 clang-format --dry-run --Werror
(for p in $PACKAGES; do
  find src/$p -name '*.cpp' -print0 | xargs -0 -n1 clang-tidy -p build/$p --quiet \
    --extra-arg="--gcc-install-dir=$(dirname "$(g++ -print-libgcc-file-name)")" || exit 1
  cppcheck --project=build/$p/compile_commands.json --std=c++20 --library=googletest \
    --enable=warning,style,performance,portability --inline-suppr --suppress=missingIncludeSystem \
    --suppress='*:/opt/ros/*' --suppress='*:/usr/include/*' --suppress='*:*/rosidl_generator_cpp/*' \
    --error-exitcode=1 --quiet || exit 1
done)
shellcheck scripts/*.sh
```

clang-tidy and cppcheck read `build/*/compile_commands.json`, so build with `-DCMAKE_EXPORT_COMPILE_COMMANDS=ON` first.

</details>

<details>
<summary>Sanitizers</summary>

Each sanitizer uses its own build tree. TSan needs `setarch -R` on kernels with high mmap randomization, and LeakSanitizer needs the ROS type-support suppression file.

```bash
A=-fsanitize=address,undefined
colcon --log-base log-asan build --build-base build-asan --install-base install-asan \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug \
  "-DCMAKE_CXX_FLAGS=$A -fno-sanitize-recover=all -fno-omit-frame-pointer -D_GLIBCXX_ASSERTIONS" \
  "-DCMAKE_EXE_LINKER_FLAGS=$A" "-DCMAKE_SHARED_LINKER_FLAGS=$A" "-DCMAKE_MODULE_LINKER_FLAGS=$A"
ASAN_OPTIONS=detect_leaks=1:abort_on_error=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 \
  LSAN_OPTIONS=suppressions=$PWD/src/rovi_tests/test/lsan.supp \
  colcon --log-base log-asan test --build-base build-asan --install-base install-asan
colcon test-result --test-result-base build-asan --all

T=-fsanitize=thread
colcon --log-base log-tsan build --build-base build-tsan --install-base install-tsan \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_CXX_FLAGS=$T -fno-omit-frame-pointer" \
  "-DCMAKE_EXE_LINKER_FLAGS=$T" "-DCMAKE_SHARED_LINKER_FLAGS=$T" "-DCMAKE_MODULE_LINKER_FLAGS=$T"
TSAN_OPTIONS=halt_on_error=1 setarch "$(uname -m)" -R \
  colcon --log-base log-tsan test --build-base build-tsan --install-base install-tsan
colcon test-result --test-result-base build-tsan --all
```

</details>

<details>
<summary>Line coverage</summary>

This combines each line's coverage across all translation units and reports production sources only.

```bash
colcon --log-base log-cov build --build-base build-cov --install-base install-cov \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug "-DCMAKE_CXX_FLAGS=--coverage -O0" \
  -DCMAKE_EXE_LINKER_FLAGS=--coverage -DCMAKE_SHARED_LINKER_FLAGS=--coverage
colcon --log-base log-cov test --build-base build-cov --install-base install-cov
for p in rovi_core rovi_telemetry rovi_scheduler rovi_network rovi_worker rovi_coordination rovi_robot_agent; do
  (cd build-cov/$p && for g in $(find CMakeFiles -name '*.gcda' | sort); do
     gcov -t -o "$(dirname "$g")" "$g" 2>/dev/null; done)
done | awk -v root="$PWD/src/" '/^ *-: *0:Source:/{f=substr($0, index($0, "0:Source:")+9); next}
  index(f, root)==1 { r=substr(f, length(root)+1); if (r ~ /\/test\//) next; split($0,p,":"); c=p[1];
    gsub(/ /,"",c); n=p[2]+0; if (c=="-" || n==0) next; k=r":"n; if (!(k in s)) s[k]=0; if (c!="#####" && c!="=====") s[k]=1 }
  END { for (k in s) { split(k,q,":"); t[q[1]]++; if (s[k]) h[q[1]]++ }
        for (f in t) { printf "%-60s %d/%d\n", f, h[f], t[f]; T+=t[f]; H+=h[f] }
        printf "TOTAL %d/%d\n", H, T }' | sort
```

</details>

## License

Apache-2.0. The fetched Panda model is CC-BY-4.0; see `src/rovi_sim/models/THIRD_PARTY.md`.
