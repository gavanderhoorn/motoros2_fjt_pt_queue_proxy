# SPDX-FileCopyrightText: 2022-2023, G.A. vd. Hoorn
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import sys
import time
import math

import rclpy

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from motoros2_interfaces.msg import QueueResultEnum
from motoros2_interfaces.srv import QueueTrajPoint
# TODO: check RobotStatus to check whether e-stops or other error
# conditions prevent the robot from actually execution the motion we've
# queued. If there are errors, abort any active goal, reject new goals.
# Similar to https://github.com/ros-industrial/industrial_core/pull/271
#from industrial_msgs.msg import RobotStatus
from sensor_msgs.msg import JointState

from simple_actions import SimpleActionServer

MAX_RETRIES_PARAM = "max_retries"
BUSY_WAIT_TIME_PARAM = "busy_wait_time"
CONVERGENCE_THRESHOLD_PARAM = "convergence_threshold"

MAX_RETRIES_DEFAULT = 20
BUSY_WAIT_TIME_DEFAULT = 0.05
CONVERGENCE_THRESHOLD_DEFAULT = 0.01

class PointQueueProxy:
    def __init__(self, node):
        self._node = node
        self._logger = self._node.get_logger()
        self._logger.info("PointQueueProxy: initialising ..")

        # Declare ROS parameters
        self._node.declare_parameter(MAX_RETRIES_PARAM, MAX_RETRIES_DEFAULT)
        self._node.declare_parameter(BUSY_WAIT_TIME_PARAM, BUSY_WAIT_TIME_DEFAULT)
        self._node.declare_parameter(CONVERGENCE_THRESHOLD_PARAM, CONVERGENCE_THRESHOLD_DEFAULT)

        # maximum nr of retries per traj pt
        try:
            self._max_retries = int(self._node.get_parameter(MAX_RETRIES_PARAM).value)
        except:
            self._logger.warning(f"Failed to load {MAX_RETRIES_PARAM} parameter, " 
                                 f"defaulting to {MAX_RETRIES_DEFAULT}")
        # seconds: how long to wait between (re)submissions
        try:
            self._busy_wait_time = float(self._node.get_parameter(BUSY_WAIT_TIME_PARAM).value)
        except:
            self._logger.warning(f"Failed to load {BUSY_WAIT_TIME_PARAM} parameter, " 
                                 f"defaulting to {BUSY_WAIT_TIME_DEFAULT}")
        # radians: total joint distance, not per-joint
        try:
            self._convergence_threshold = float(self._node.get_parameter(CONVERGENCE_THRESHOLD_PARAM).value)
        except:
            self._logger.warning(f"Failed to load {CONVERGENCE_THRESHOLD_PARAM} parameter, " 
                                 f"defaulting to {CONVERGENCE_THRESHOLD_DEFAULT}")

        # TODO: use remapping, not parameters
        self._joint_states_topic: str = 'joint_states'
        self._fjt_namespace: str = 'joint_trajectory_controller'
        self._fjt_name: str = 'follow_joint_trajectory'
        self._queue_pt_srv: str = 'queue_traj_point'

        # first the service client, as there's no point in continuing if the
        # server is not available
        self._cbg_svc = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._queue_pt_client = self._node.create_client(
            QueueTrajPoint, self._queue_pt_srv, callback_group=self._cbg_svc)
        while not self._queue_pt_client.wait_for_service(timeout_sec=5.0):
            self._logger.info('Waiting for queue_traj_point server ..')

        # TODO: see whether this needs its own callback group (if yes: can't
        # use simple_actions any more I believe)
        fjt_server_ns = f'{self._fjt_namespace}/{self._fjt_name}'
        self._logger.debug(f"Starting action server on '{fjt_server_ns}'")
        self._action_server = SimpleActionServer(
            self._node, FollowJointTrajectory,
            fjt_server_ns,
            self.fjt_goal_callback)

        # MotoROS2 might be using 'sensor_data' profile or 'default'.
        # Use 'sensor_data' here, as it should be compatible with both
        self._sub_js = self._node.create_subscription(
            JointState, self._joint_states_topic, self._js_callback,
            qos_profile=rclpy.qos.QoSPresetProfiles.get_from_short_key('sensor_data'),
            callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup())

        # stores last message we received from controller
        self._latest_jstates: JointState = None

        self._logger.info("PointQueueProxy: initialisation complete")


    def _queue_point(self, joint_names: list[str], pt: JointTrajectoryPoint, max_retries: int = 0):
        self._logger.debug(f"attempting to queue pt (max_retries: {max_retries})")

        attempts: int = 0
        result_code: int = -1
        req = QueueTrajPoint.Request(joint_names=joint_names, point=pt)

        # TODO: ugly ternary
        while rclpy.ok() and ((attempts < max_retries) if max_retries else True):
            self._logger.debug(f"queuing pt (attempt: {attempts})")

            # use a sync request to keep control flow 'simple'.
            # This should work as we use different cb grps and an mt executor.
            # TODO: the synchronous call does not support setting a timeout,
            # so if the server dies/disappears, this hangs forever
            response = self._queue_pt_client.call(req)

            # only if we receive a BUSY response we try again. Anything else
            # is something only the caller can handle (including OK)
            result_code = response.result_code.value
            if result_code == QueueResultEnum.BUSY:
                self._logger.debug(
                    f"Busy (attempt: {attempts}). Trying again later.", throttle_duration_sec=1.0)
                attempts += 1
                time.sleep(self._busy_wait_time)
                continue

            if result_code == QueueResultEnum.SUCCESS:
                self._logger.debug(
                    f"queue server accepted point: '{response.message}' ({result_code})")
                break

            # anything else is an error, so report
            self._logger.warning(
                f"queue server returned error: '{response.message}' ({result_code})")
            break

        # either -1, or one of the values from QueueResultEnum
        return result_code


    def _js_callback(self, msg):
        # TODO: add locking (if needed)
        self._latest_jstates = msg

        # NOTE: this does not work with upstream 'simple_actions', as it
        #       treats is_active as a method on the goal handle (it's a property).
        #       See https://github.com/DLu/simple_actions/issues/1
        if not self._action_server.is_active() or not self._action_server.is_executing():
            return

        fmsg = FollowJointTrajectory.Feedback()

        # populate fields using latest JointState information we received, as
        # an approximation of the feedback MotoROS2 would send back during the
        # execution of an FJT goal
        fmsg.header.stamp = msg.header.stamp
        fmsg.joint_names = msg.name
        fmsg.actual.positions = msg.position
        fmsg.actual.velocities = msg.velocity
        fmsg.actual.effort = msg.effort

        self._action_server.publish_feedback(fmsg)


    def _joint_distance(self, d0: dict[str, float], d1: dict[str, float]) -> float:
        # assumptions: d1 contains all keys d0 contains
        assert len(d0) == len(d1)
        return math.fsum([abs(val - d1[name]) for name, val in d0.items()])


    def fjt_goal_callback(self, goal):
        self._logger.debug('fjt callback: entry')

        traj = goal.trajectory
        points = traj.points
        points_sent: int = 0

        self._logger.debug(f"received goal with {len(points)} traj pts")

        # checks
        # TODO: add locking (if needed)
        if not self._latest_jstates:
            error_string = "waiting for (initial) feedback from controller"
            self._logger.error(error_string)
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string=error_string)

        if len(points) == 0:
            # TODO: implement motoman_driver/industrial_robot_client behaviour
            # (ie: cancel any executing trajectory)
            error_string = "not executing an empty trajectory"
            self._logger.warning(error_string)
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL,
                error_string=error_string)

        if len(traj.joint_names) == 0:
            error_string = "no joint names, can't continue"
            self._logger.error(error_string)
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_JOINTS,
                error_string=error_string)

        # arbitrary, but there aren't (m)any Motoman robots with less than
        # four joints, especially not ones supported by MotoROS2
        if len(traj.joint_names) < 4:
            self._logger.warning("less than 4 joint names")

        # TODO: we could/should also check whether joint names in the goal
        # correspond to the MotoROS2 configured joint names, but we have no
        # way of accessing MotoROS2's configuration at the moment.
        # (could potentially sample 'joint_states' topic and use those names)

        # we're going to process the goal, so relay JointStates published
        # by MotoROS2 as FollowJointTrajectory_Feedback

        # iterate over all points, starting with the first. Convert each point
        # to a queue request, then send it off. If not success, repeat until
        # we've reached our time-out value.
        while rclpy.ok() and (points_sent < len(points)):
            self._logger.debug(f"attempting to queue pt {points_sent}")

            # TODO: check whether goal has been cancelled in the meantime
            # TODO: check whether js watchdog has bitten and cancel/abort goal ourselves

            pt = points[points_sent]
            result = self._queue_point(
                joint_names=traj.joint_names, pt=pt, max_retries=self._max_retries)

            # if this is an error, or still BUSY, something is wrong. Abort
            # the goal and report error
            if result != QueueResultEnum.SUCCESS:
                error_string = (f"failed to queue pt {points_sent}, aborting goal "
                                f"(queue server reported: {result})")
                self._logger.error(error_string)
                return FollowJointTrajectory.Result(
                    # TODO: use MotoROS2 error reporting method
                    error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                    error_string=error_string)

            self._logger.info(f"pt {points_sent} queued")

            # next pt
            points_sent += 1

        # done
        self._logger.info("queued all points")

        # now we wait until MotoROS2 reports it has reached the final traj pt.
        # We do that by comparing the current JointStates against the final
        # trajectory point in the trajectory submitted as part of the goal.
        # As soon as the distance is below the threshold, we assume the traj
        # has completely executed
        #
        # NOTE: this approach suffers from the exact same problems as the
        # FJT action server in industrial_robot_client (looping traj, etc)
        self._logger.info(
            "waiting for robot to reach final traj pt "
            f"(threshold: {self._convergence_threshold} rad)")

        # TODO: add a timeout. If JointStates haven't converged within the
        # timeout, consider goal to have failed (regardless of whether the
        # pts were successfully queued).
        # Would also need to make sure to cancel any active motion

        # TODO: add locking (if needed)
        if not self._latest_jstates:
            self._logger.warning("Can't track progress as no joint "
                "states received, not waiting for execution before reporting "
                "success")
        else:
            last_traj_dict = dict(zip(traj.joint_names, points[-1].positions))
            rate = self._node.create_rate(30.0)
            while rclpy.ok():
                # TODO: add locking (if needed)
                js_dict = dict(zip(
                    self._latest_jstates.name, self._latest_jstates.position))
                dist = self._joint_distance(last_traj_dict, js_dict)
                self._logger.debug(
                    f"remaining distance: {dist:.4f}", throttle_duration_sec=1)
                if dist <= self._convergence_threshold:
                    self._logger.info(
                        f"reached final traj pt (distance: {dist:.4f})")
                    break
                rate.sleep()

        # done executing the trajectory, so report the result
        # TODO: result could be negative if there was an error (RobotStatus),
        # it takes too long to reach the last traj pt, etc.
        result = FollowJointTrajectory.Result(
            error_code=FollowJointTrajectory.Result.SUCCESSFUL,
            error_string="")

        self._logger.debug('fjt callback: exit')
        return result


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.node.Node('motoros2_fjt_pt_queue_proxy')
    server = PointQueueProxy(node)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
