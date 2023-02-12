# CMPUT 412 Exercise 2 - Part 1
This repository contains all the code for Exercise 2 Part 1.

## Usage
1. Clone the repository.
2. Modify launch file to run desired node.
3. Build `dts devel build -H $BOT`.
4. Run `dts devel run -H $BOT`.

## Nodes
* odometry_node: publish intergrated travelled distance info under topic `~dist_l` and `~dist_r`.
* my_subscriber_node: a custom subscriber node.
* my_publisher_node: a custom publisher node.
* my_camera_node: subscribe to the camera feed and publish vertically mirrored image from it (`~image/compressed`).


## License
[Duckietown](https://www.duckietown.org/about/sw-license)
