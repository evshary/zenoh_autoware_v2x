# zenoh_autoware_v2x

![](resource/Autoware_V2X_Zenoh_Architecture.svg)


## Setup

> [!IMPORTANT]
> Make sure you have already cloned this repo into autoware_carla_launch/external.

```bash=
(in bridge container...)
$ cd zenoh_autoware_v2x && poetry install --no-root

(in Autoware container...)
$ cd zenoh_autoware_v2x && colcon build --symlink-install
$ pip install eclipse-zenoh==0.10.0rc0
```

## Run

- in bridge container

```bash=
cd autoware_carla_launch
source env.sh
./script/run-bridge-v2x.sh
```

- in Autoware container

```bash=
cd autoware_carla_launch
source env.sh
./script/run-autoware.sh

(Open another window in autoware container for executing the below commands)
source external/zenoh_autoware_v2x/install/setup.bash
ros2 run v2x_light v2x_light -- -v <vehicle_ID>
```
