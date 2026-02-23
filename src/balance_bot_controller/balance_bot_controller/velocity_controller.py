#!/usr/bin/env python3

import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64
WHEEL_RADIUS = 0.03
class VelocityController(Node):
    def __init__(self):
        super().__init__("Velocity_Controller")

        self.desired_setpoint_error = self.create_publisher(Float64, "/desired_setpoint/error", 10)
        self.velocity_error_ = self.create_publisher(Float64, "/velocity/error", 10)
        self.estimated_velocity_ = self.create_publisher(Float64, "/estimated/vel", 10)

        self.joint_states = self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 10)

        self.setpoint_pub = self.create_publisher(Float64, "/desired_setpoint", 10)
        self.timer = self.create_timer(0.005, self.setpoint_cmd)

        self.estimated_velocity = 0
        self.angular_wheel_L = 0
        self.angular_wheel_R = 0

        ## Still
        self.desired_velocity = 0
        self.velocity_error = 0


        self.Kv = 0.2

        self.min_pitch = -0.15
        self.max_pitch = 0.15

    def joint_states_cb(self, joint_state:JointState):
        self.angular_wheel_L = joint_state.velocity[0]
        self.angular_wheel_R = joint_state.velocity[1]
    
    def setpoint_cmd(self):
        setpoint_reference = Float64()
        ## -Wheel radius due to the velocity being -ve when pitch is forward
        ## This can be changed in the URDF
        self.estimated_velocity = -WHEEL_RADIUS * ((self.angular_wheel_L + self.angular_wheel_R) / 2)
        self.velocity_error = self.desired_velocity - self.estimated_velocity

        pitch_setpoint = self.Kv * self.velocity_error
        
        if pitch_setpoint >= self.max_pitch:
            pitch_setpoint = self.max_pitch
        elif pitch_setpoint <= self.min_pitch:
            pitch_setpoint = self.min_pitch

        setpoint_reference.data = pitch_setpoint
        self.setpoint_pub.publish(setpoint_reference)

        estimated_velocity = Float64()
        velocity_error = Float64()
        estimated_velocity.data = self.estimated_velocity
        velocity_error.data = self.velocity_error

        self.estimated_velocity_.publish(estimated_velocity)
        self.velocity_error_.publish(velocity_error)
        self.desired_setpoint_error.publish(setpoint_reference)




def main():
    rclpy.init()
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
