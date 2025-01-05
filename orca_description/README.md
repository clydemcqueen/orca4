# Orca4 SDF Files

Orca4 uses the worlds and models from [bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz).

The [bluerov2](https://github.com/clydemcqueen/bluerov2_gz/tree/main/models/bluerov2) model
was copied and modified to create the [orca4](models/orca4) model.

The [underwater](https://github.com/clydemcqueen/bluerov2_gz/tree/main/worlds/underwater) world
was copied and modified to create the [sand](models/orca4) world.

The [generate_model.py](scripts/generate_model.py) script takes the [orca/model.sdf.in](models/orca4/model.sdf.in)
file and replaces strings of the form `@foo` with calculated values to generate
[orca/model.sdf](models/orca4/model.sdf).
The script must be run manually when the model is changed.

An URDF file is not provided. In practice this means 2 things:
1. static transforms are published by instances `tf2_ros/static_transform_publisher`
2. Rviz2 will only show transforms (Rviz2 can't read SDF)