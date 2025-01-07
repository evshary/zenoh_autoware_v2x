# Zenoh Autoware V2X

## Architecture

![V2X Architecture](resource/Autoware_V2X_Zenoh_Architecture.svg)

## Sequence Diagram

```mermaid
sequenceDiagram
    participant tm as Traffic Manager
    participant im as Intersection Manager

    im ->> tm: Load lane_to_light map

    par Normal Traffic Light Operation
        loop Intersection A
            im ->> im: Automatically change signals periodically for Intersection A
        end

        loop Intersection B
            im ->> im: Automatically change signals periodically for Intersection B
        end

        loop Intersection N
            im ->> im: Automatically change signals periodically for Intersection N
        end
    end

    tm ->> tm: Subscribe to high priority vehicle positions

    alt High priority vehicle enters 20m radius of any intersection
        tm ->> im: Change signals to allow vehicle to pass
            im ->> im: Pause automatic signal change for affected intersection
            im ->> im: Check current intersection state
            alt Cars in intersection
                im ->> im: Set red light to stop incoming cars, let cars in intersection pass
            end
            im ->> im: Set green light for high priority vehicle
    end

    alt High priority vehicle passes through the intersection
        tm ->> im: Restore original traffic light signals
        
            im ->> im: Resume automatic signal change for affected intersection
        
    end

    Note over tm, im: Handle emergency vehicles, normal vehicles follow standard traffic lights

```

## Setup

> [!IMPORTANT]
> Please clone this repository under `autoware_carla_launch/external/` first.

- in Autoware container

```shell
cd autoware_carla_launch/external/zenoh_autoware_v2x
colcon build --symlink-install
```

## Run

- Run Carla simulator (In native host)

```shell
./CarlaUE4.sh -quality-level=Epic -world-port=2000 -RenderOffScreen -prefernvidia
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
./script/run-autoware.sh

# Open another window in the Autoware container to execute the commands below.
source external/zenoh_autoware_v2x/install/setup.bash
ros2 run v2x_light v2x_light -- -v <vehicle_ID>
```

**Note:** <vehicle_id> must same as CARLA agent's rolename. (default is "v1")

## For Developers

You can use pre-commit and Ruff to have correct Python format

```shell
python3 -m pip install pre-commit ruff
pre-commit install --install-hooks
```
