# motoros2_fjt_pt_queue_proxy

Drip-feed `FollowJointTrajectory` goals to MotoROS2 one `JointTrajectoryPoint` at a time.

This is essentially a Python implementation of the `FollowJointTrajectory` action server in `industrial_robot_client` / `motoman_driver`.


## Status

WIP.

The node currently suffers a *cpu leak*, in the sense that consecutive goals cause the node to consume increasingly large amounts of CPU time, until a full core is consumed.
