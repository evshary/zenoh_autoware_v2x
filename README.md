# zenoh_autoware_v2x

![](resource/Autoware_V2X_Zenoh_Architecture.svg)


## Setup

> [!IMPORTANT]
> Please clone this repository under `autoware_carla_launch/external/` first.

- in bridge container
```shell
cd autoware_carla_launch
source env.sh
cd external/zenoh_autoware_v2x
poetry env use $(pyenv which python) && poetry install --no-root
```
- in Autoware container
```shell
cd autoware_carla_launch/external/zenoh_autoware_v2x
pip install -r requirements.txt
colcon build --symlink-install
```

## Run
- Run Carla simulator (In native host)

```shell
./CarlaUE4.sh -quality-level=Epic -world-port=2000 -RenderOffScreen
```
- in bridge container

```shell
cd autoware_carla_launch
source env.sh
./script/run-bridge-v2x.sh
```

- in Autoware container

```shell
cd autoware_carla_launch
source env.sh
# For the reinstallation of the eclipse-zenoh package each time the Autoware container is executed.
pip install -r external/zenoh_autoware_v2x/requirements.txt
./script/run-autoware.sh

# Open another window in the Autoware container to execute the commands below.
source external/zenoh_autoware_v2x/install/setup.bash
ros2 run v2x_light v2x_light -- -v <vehicle_ID>
```
**Note:** <vehicle_id> must same as CARLA agent's rolename. (default is "v1")
