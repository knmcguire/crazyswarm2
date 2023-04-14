import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from math import cos, sin, degrees, radians, pi
import sys
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


from crazyflie_interfaces.msg import FullState, Commands

# Change this path to your crazyflie-firmware folder
sys.path.append('/home/knmcguire/Development/bitcraze/c/crazyflie-firmware')
import cffirmware



import numpy as np

class CrazyflieWebotsDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        timestep = int(self.robot.getBasicTimeStep())

        ## Initialize motors
        self.m1_motor = self.robot.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.robot.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.robot.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.robot.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)


        ## Initialize Sensors
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(timestep)
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(timestep)

        ## Intialize Variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_z_global = 0
        self.past_time = self.robot.getTime()

        self.first_pos = True
        self.first_x_global = 0.0
        self.first_y_global = 0.0

        self.target_cmds = np.array([0, 0, 0, 0])

        cffirmware.controllerPidInit()

        rclpy.init(args=None)
        self.node = rclpy.create_node('crazyflie_webots_driver')

        self.node.create_subscription(
            FullState, '/cf231/desired_state', self.desired_state_callback, 1)
        self.node.create_subscription(
            Commands, '/cf231/commands', self.command_callback, 1)
        self.target_state = FullState()
        self.state_publisher = self.node.create_publisher(FullState, '/cf231/next_state', 1)

        self.node.get_logger().info("state info")

    def desired_state_callback(self, msg):
        self.target_state = msg

        self.node.get_logger().info(f"{msg.pose.orientation.x} {msg.pose.orientation.y} {msg.pose.orientation.z} {msg.pose.orientation.w}")

    def command_callback(self, msg):
        self.target_cmds = np.array([msg.roll, msg.pitch, msg.yaw, msg.thrust])   
        self.node.get_logger().info(f"{msg.roll} {msg.pitch} {msg.yaw} {msg.thrust}")


    def step(self):

        rclpy.spin_once(self.node, timeout_sec=0)


        dt = self.robot.getTime() - self.past_time

        if self.first_pos is True:
            self.first_x_global = self.gps.getValues()[0]
            self.first_y_global = self.gps.getValues()[1]
            self.first_pos = False

        ## Get measurements
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        roll_rate = self.gyro.getValues()[0]
        pitch_rate = self.gyro.getValues()[1]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0]- self.first_x_global
        vx_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1] - self.first_y_global
        vy_global = (y_global - self.past_y_global)/dt
        z_global = self.gps.getValues()[2]
        vz_global = (z_global - self.past_z_global)/dt

        msg = FullState()
        msg.pose.position.x = x_global
        msg.pose.position.y = y_global
        msg.pose.position.z = z_global
        # roll pitch yaw to quarternion x y z w
        q_base = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q_base[0]
        msg.pose.orientation.y = q_base[1]
        msg.pose.orientation.z = q_base[2]
        msg.pose.orientation.w = q_base[3]

        msg.twist.linear.x = vx_global
        msg.twist.linear.y = vy_global
        msg.twist.linear.z = vz_global
        msg.twist.angular.x = roll_rate
        msg.twist.angular.y = pitch_rate
        msg.twist.angular.z = yaw_rate

        self.state_publisher.publish(msg)

        ## Put measurement in state estimate
        # TODO replace these with a EKF python binding
        state = cffirmware.state_t()
        state.attitude.roll = degrees(roll)
        state.attitude.pitch = -degrees(pitch)
        state.attitude.yaw = degrees(yaw)
        state.position.x = x_global
        state.position.y = y_global
        state.position.z = z_global
        state.velocity.x = vx_global
        state.velocity.y = vy_global
        state.velocity.z = vz_global
        
        # Put gyro in sensor data
        sensors = cffirmware.sensorData_t()
        sensors.gyro.x = degrees(roll_rate)
        sensors.gyro.y = degrees(pitch_rate)
        sensors.gyro.z = degrees(yaw_rate)
        yawDesired=0

        ## Fill in Setpoints
        setpoint = cffirmware.setpoint_t()
        setpoint.mode.yaw = cffirmware.modeVelocity

        # TODO: find out why this multipication is necessary...
        setpoint.mode.yaw = cffirmware.modeVelocity
        setpoint.attitudeRate.yaw = degrees(self.target_state.twist.angular.z)*5
        setpoint.mode.x = cffirmware.modeAbs
        setpoint.mode.y = cffirmware.modeAbs
        setpoint.mode.z = cffirmware.modeAbs
        setpoint.position.x = self.target_state.pose.position.x
        setpoint.position.y = self.target_state.pose.position.y
        setpoint.position.z = self.target_state.pose.position.z


        ## Firmware PID bindings
        control = cffirmware.control_t()
        tick = 100 #this value makes sure that the position controller and attitude controller are always always initiated
        cffirmware.controllerPid(control, setpoint,sensors,state,tick)

        ## 
        cmd_roll = radians(control.roll)
        cmd_pitch = radians(control.pitch)
        cmd_yaw = -radians(control.yaw)
        cmd_thrust = control.thrust

        #cmd_roll = radians(self.target_cmds[0])
        #cmd_pitch = radians(self.target_cmds[1])
        #cmd_yaw = -radians(self.target_cmds[2])
        #cmd_thrust = self.target_cmds[3]

        ## Motor mixing
        motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
        motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
        motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
        motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw

        scaling = 900 ##Todo, remove necessity of this scaling (SI units in firmware)
        self.m1_motor.setVelocity(-motorPower_m1/scaling)
        self.m2_motor.setVelocity(motorPower_m2/scaling)
        self.m3_motor.setVelocity(-motorPower_m3/scaling)
        self.m4_motor.setVelocity(motorPower_m4/scaling)

        self.past_time = self.robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global
        self.past_z_global = z_global
