#!/bin/bash
# perf_check.sh: find out what keeps the sim below real time on this machine.
#
# Runs the sim several ways for ~20 s each and prints the real-time factor (RTF) of each,
# averaged over the whole window (the live real_time_factor readout is too jumpy to judge):
#   0    physics with no real-time limit: how many times faster than real time this CPU
#        can run it, free to use every CPU and pinned to 2 (what sim.launch.py does on WSL)
#   1-4  Gazebo alone (no ROS nodes): headless or with the GUI, cameras off or rendering
#   5-8  the full sim.launch.py: headless, normal, without CPU pinning, with a 4 ms physics step
# Comparing rows shows what costs the most: the GUI, the cameras, or the ROS nodes.
# Gazebo and keyboard windows open and close by themselves while it runs (~5 min).
#
#   source /opt/ros/$ROS_DISTRO/setup.bash && source install/setup.bash
#   src/mate_rov_sim/tools/perf_check.sh

WARMUP=${WARMUP:-12}   # seconds to let everything start before measuring
MEASURE=${MEASURE:-15}

if ! ros2 pkg prefix mate_rov_sim >/dev/null 2>&1; then
    echo "Source ROS and the workspace first (source install/setup.bash)."; exit 1
fi
SHARE=$(ros2 pkg prefix mate_rov_sim)/share/mate_rov_sim
WORLD=$SHARE/worlds/mate_pool.sdf
export GZ_SIM_RESOURCE_PATH=$SHARE/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}
export ROS_DOMAIN_ID=$((RANDOM % 100 + 100))   # don't talk to any other running sim
RENDER=ogre2
if grep -qi microsoft /proc/version; then
    RENDER=ogre
    # same GPU selection as sim.launch.py
    [ -e /usr/lib/wsl/lib/libd3d12.so ] && export GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12} \
        MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}
fi

sim_clock() {   # prints "sim_seconds real_seconds" from one stats message
    timeout 5 gz topic -e -t /world/mate_pool/stats -n 1 2>/dev/null | awk '
        /^sim_time/ {m = "s"} /^real_time \{/ {m = "r"}
        /^ +sec:/ {v[m] += $2} /^ +nsec:/ {v[m] += $2 / 1e9}
        END {print v["s"] + 0, v["r"] + 0}'
}

stop_partition() {   # kill everything started under this GZ_PARTITION (gz forks its own children)
    for pid in $(pgrep -u "$USER"); do
        grep -qz "^GZ_PARTITION=$1\$" /proc/$pid/environ 2>/dev/null && kill -INT $pid 2>/dev/null
    done
    sleep 3
    for pid in $(pgrep -u "$USER"); do
        grep -qz "^GZ_PARTITION=$1\$" /proc/$pid/environ 2>/dev/null && kill -9 $pid 2>/dev/null
    done
}

# run_case <label> <cameras: 0|1> <command...>
run_case() {
    local label=$1 cams=$2; shift 2
    export GZ_PARTITION=perf$$_$RANDOM
    setsid "$@" >/tmp/perf_check_last.log 2>&1 &
    disown   # gz sim can segfault on shutdown; don't print that
    sleep "$WARMUP"
    if [ "$cams" = 1 ]; then   # a subscriber makes Gazebo render the cameras, like the bridge does
        gz topic -e -t /rov/camera_0/image >/dev/null 2>&1 &
        gz topic -e -t /rov/camera_1/image >/dev/null 2>&1 &
        sleep 3
    fi
    read -r s1 r1 < <(sim_clock)
    sleep "$MEASURE"
    read -r s2 r2 < <(sim_clock)
    stop_partition "$GZ_PARTITION"
    wait 2>/dev/null
    awk -v l="$label" -v a="$s1" -v b="$s2" -v c="$r1" -v d="$r2" 'BEGIN {
        if (d > c && b > a) printf "  %-44s RTF %.2f\n", l, (b - a) / (d - c);
        else printf "  %-44s failed (see /tmp/perf_check_last.log)\n", l }'
}

echo "CPU:    $(lscpu | sed -n 's/^Model name: *//p') ($(nproc) threads)"
echo "Memory: $(free -h | awk '/^Mem/ {print $2 " total, " $7 " available"}')"
echo "Render: $RENDER, GALLIUM_DRIVER=${GALLIUM_DRIVER:-<unset>}"
echo
GZ=(gz sim -r -v 1 --render-engine $RENDER "$WORLD")
UNLIMITED=$(mktemp --suffix=.sdf)   # same world, real_time_factor 0 (run flat out), no cameras
sed 's#<real_time_factor>[^<]*</real_time_factor>#<real_time_factor>0</real_time_factor>#; /gz-sim-sensors-system/d' \
    "$WORLD" > "$UNLIMITED"
LAST2=$(taskset -cp $$ | sed 's/.*: //' | tr ',' '\n' | awk -F- '{for (c = $1; c <= ($2 == "" ? $1 : $2); c++) print c}' | tail -2 | paste -sd,)
run_case "0a physics capacity, all CPUs"             0 gz sim -r -s -v 1 "$UNLIMITED"
run_case "0b physics capacity, pinned to CPUs $LAST2" 0 taskset -c "$LAST2" gz sim -r -s -v 1 "$UNLIMITED"
rm -f "$UNLIMITED"
run_case "1 Gazebo headless, cameras off (physics)"  0 "${GZ[@]}" -s
run_case "2 Gazebo headless, cameras rendering"      1 "${GZ[@]}" -s
run_case "3 Gazebo GUI, cameras off"                 0 "${GZ[@]}"
run_case "4 Gazebo GUI, cameras rendering"           1 "${GZ[@]}"
run_case "5 full sim.launch.py, headless"            0 ros2 launch mate_rov_sim sim.launch.py "gz_args:=-r -s -v 1"
run_case "6 full sim.launch.py (normal)"             0 ros2 launch mate_rov_sim sim.launch.py
run_case "7 full sim.launch.py, gz_cpus:=all"        0 ros2 launch mate_rov_sim sim.launch.py gz_cpus:=all
run_case "8 full sim.launch.py, physics_step:=0.004" 0 ros2 launch mate_rov_sim sim.launch.py physics_step:=0.004
echo
LOG=$(ls -t ~/.gz/rendering/ogre*.log 2>/dev/null | head -1)
echo "GPU used: $(grep -h GL_RENDERER "$LOG" 2>/dev/null | tail -1 | sed 's/.*= //')"
