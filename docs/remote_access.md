# Running the workspace over SSH

The bringup has to run on the workstation cabled to X66. FRI is a 2 ms UDP loop and
does not tolerate an extra network hop, so the ROS side is not something to run from
your laptop across the lab network.

## Basic session

```bash
ssh <user>@<workstation>
cd path/to/pixi_kuka_ros2
pixi run -e jazzy hardware
```

## Use a multiplexer

Bringup, status, and any move each need their own shell, and the bringup must outlive
the SSH connection while you walk to the smartPAD to answer `LBRServer`'s four
dialogs. A dropped SSH session takes the FRI session with it otherwise.

```bash
ssh <user>@<workstation>
tmux new -s kuka                    # or: tmux attach -t kuka
pixi run -e jazzy hardware          # leave running; detach with Ctrl-b d
```

In a second window (`Ctrl-b c`) on the same host:

```bash
pixi run -e jazzy session           # status only, safe to run at any time
pixi run -e jazzy controllers
```

## Subscribing from another machine

To read topics from a different machine on the same lab network, match the DDS
settings that `scripts/set_ros_env.sh` exports:

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo /lbr/joint_states
```

Both must match or discovery silently finds nothing.

Do not use `mock-loopback` when you need to reach the workspace from another host: it
pins CycloneDDS to the loopback interface via `config/cyclonedds_loopback.xml`, which
is what you want for a single-host or sandbox test and exactly what you do not want
here.

## Note

The commands above are derived from `pixi.toml` and `scripts/set_ros_env.sh`. The
workspace path and the availability of `tmux` depend on how the workstation was
provisioned — confirm both on the machine itself.
