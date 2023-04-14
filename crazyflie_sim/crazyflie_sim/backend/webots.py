from __future__ import annotations

from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from ..sim_data_types import State, Action

from crazyflie_interfaces.msg import FullState, Commands

from functools import partial
import numpy as np

class Backend:
    """Tracks the desired state perfectly (no physics simulation)"""

    def __init__(self, node: Node, names: list[str], states: list[State]):
        self.node = node
        self.names = names
        self.clock_publisher = node.create_publisher(Clock, 'clock', 10)
        self.t = 0
        self.dt = 0.1

        self.cmd_publishers = dict()
        self.state_publishers = dict()
        self.subscriptions = dict()
        self.next_states = dict()
        for name in names:
            self.cmd_publishers[name] = node.create_publisher(Commands, name + '/commands', 10)
            self.state_publishers[name] = node.create_publisher(FullState, name + '/desired_state', 10)
            self.subscriptions[name] = node.create_subscription(FullState, name + '/next_state', partial(self.next_state_callback, name=name), 10)
            self.next_states[name] = State()

    def next_state_callback(self, msg, name='cf231'):
        self.next_states[name] = State(
            pos=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]),
            vel=np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]),
            quat=np.array([msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z]),
            omega=np.array([msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z]))


    def time(self) -> float:
        return self.t

    def step(self, states_desired: list[State], actions: list[Action]) -> list[State]:
        # advance the time
        self.t += self.dt

        # publish the current clock
        #clock_message = Clock()
        #clock_message.clock = Time(seconds=self.time()).to_msg()
        #self.clock_publisher.publish(clock_message)
        for action, name in zip(actions, self.names):
            self.node.get_logger().info(f"{action.rpm} {action.cmd}")
            msg = Commands()
            msg.header.stamp = Time(seconds=self.time()).to_msg()
            msg.roll = action.cmd[0]
            msg.pitch = action.cmd[1]
            msg.yaw = action.cmd[2]
            msg.thrust = action.cmd[3]
            msg.m1rpm = action.rpm[0]
            msg.m2rpm = action.rpm[1]
            msg.m3rpm = action.rpm[2]
            msg.m4rpm = action.rpm[3]

            self.cmd_publishers[name].publish(msg)

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
            self.state_publishers[name].publish(msg)
        
        # return the next states
        return [self.next_states[name] for name in self.names]

    def shutdown(self):
        pass




