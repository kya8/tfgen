# Intro

`tfgen` is a simple command-line application for building and querying graph of rigid transforms.

Think `transform_graph` in ROS, but as a standalone utility.

# Usage
## Add transforms
Transform nodes are identified by names. Add a transform by typing:
```
<source_name> -> <target_name> : <transform>
```
E.g.:
```
cam -> lidar : 1,0,-0.5,1,0,0,0
```
A transform can be specified in multiple ways:
* $t_x, t_y, t_z, q_x, q_y, q_z, q_w$
* $t_x, t_y, t_z$ (translation only)
* $q_x, q_y, q_z, q_w$ (rotation only)
* $m_{00}, m_{01}, m_{02}, m_{10}, m_{11}, m_{12}, m_{20}, m_{21}, m_{22}$ (3x3 matrix in row-major order. Rotation only)
* $m_{00}, m_{01}, m_{02}, m_{03}, m_{10}, m_{11}, m_{12}, m_{13}, m_{20}, m_{21}, m_{22}, m_{23}, m_{30}, m_{31}, m_{32}, m_{33}$ (4x4 matrix in row-major order)
* random: Generates a random transform with uniformly sampled rotation from $SO(3)$

Add another transform:
```
imu -> lidar : 1, 2, 3, 0, 0, 0, 1
```

### Cyclic transforms

Cyclic transforms are not allowed. That is, adding a transform which would form a cycle with existing transforms is rejected. This includes self-cycles (`a->a`). This restriction guarantees a unique transform path.

## Query transform
Query a transform by typing:
```
<source_name> -> <target_name>
```
The resulting transform, if any, will be printed alongside with its path.

E.g.:
```
cam -> imu
Transform from cam to imu: (Path: cam -> lidar -> imu)
┌                     ┐
│    1    0    0    0 │
│    0   -1    0   -2 │
│    0    0   -1 -3.5 │
│    0    0    0    1 │
└                     ┘
[x,y,z, qx,qy,qz,qw]: [0.0, -2.0, -3.5, 1.0, 0.0, 0.0, 0.0]
```

## Save and load transform graph
Transforms can be serialized/deserialized to JSON with `save|load <filename>.json`.
