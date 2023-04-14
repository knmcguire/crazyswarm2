from __future__ import annotations

from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from ..sim_data_types import State, Action

from crazyflie_interfaces.msg import FullState


class Backend:
    """Tracks the desired state perfectly (no physics simulation)"""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 0.1

        self.publishers = dict()
        for name in names:
            self.publishers[name] = node.create_publisher(FullState, name + '/desired_state', 10)


    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        # advance the time
        self.t += self.dt

        # publish the current clock
        clock_message = Clock()
        clock_message.clock = Time(seconds=self.time()).to_msg()
        self.clock_publisher.publish(clock_message)

        for name, state in zip(self.names, states_desired):
            self.node.get_logger().info(f"{name} {state.pos} {state.vel} {state.quat} {state.omega}")
            msg = FullState()
            msg.header.stamp = Time(seconds=self.time()).to_msg()
            msg.pose.position.x = state.pos[0]
            msg.pose.position.y = state.pos[1]
            msg.pose.position.z = state.pos[2]
            msg.pose.orientation.x = state.quat[1]
            msg.pose.orientation.y = state.quat[2]
            msg.pose.orientation.z = state.quat[3]
            msg.pose.orientation.w = state.quat[0]
            msg.twist.linear.x = state.vel[0]
            msg.twist.linear.y = state.vel[1]
            msg.twist.linear.z = state.vel[2]
            msg.twist.angular.x = state.omega[0]
            msg.twist.angular.y = state.omega[1]
            msg.twist.angular.z = state.omega[2]
            self.publishers[name].publish(msg)

        # pretend we were able to follow desired states perfectly
        return states_desired

    def shutdown(self):
        pass




