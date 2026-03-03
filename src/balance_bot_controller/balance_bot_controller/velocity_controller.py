#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Float64
WHEEL_RADIUS = 0.03


class VelocityController(Node):
    def __init__(self):
        super().__init__("Velocity_Controller")

        self.desired_setpoint_error = self.create_publisher(
            Float64, "/desired_setpoint/error", 10)
        self.velocity_error_ = self.create_publisher(
            Float64, "/velocity/error", 10)
        self.estimated_velocity_ = self.create_publisher(
            Float64, "/estimated/vel", 10)
        self.setpoint_pub = self.create_publisher(
            Float64, "/desired_setpoint", 10)

        self.joint_states = self.create_subscription(
            JointState, "/joint_states", self.joint_states_cb, 10)

        self.timer = self.create_timer(0.005, self.setpoint_cmd)

        self.angular_wheel_L = 0
        self.angular_wheel_R = 0

        # Still
        self.desired_velocity = 0

        self.velocity_error = 0
        self.estimated_velocity = 0

        self.min_pitch = -0.15
        self.max_pitch = 0.15

        self.declare_parameter("Kv", 0.2)
        self.declare_parameter("Ki", 0.01)
        self.Kv = self.get_parameter("Kv").get_parameter_value().double_value
        self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
        self.integral = 0
        self.add_on_set_parameters_callback(
            self.event_callback)
        
        self.last_time = None

    def event_callback(self, parameter):
        result = SetParametersResult()
        for p in parameter:
            if p.name in ("Kv", "Ki"):
                if p.value < 0.0:
                    result.successful = False
                    result.reason = f"Invalid Parameter Update: {p.name}: {p.value}"
                    return result
            
            if p.name == "Kv":
                self.Kv = p.value
            elif p.name == "Ki":
                self.Ki = p.value

            result.successful = True
            return result

    def joint_states_cb(self, joint_state: JointState):
        self.angular_wheel_L = joint_state.velocity[0]
        self.angular_wheel_R = joint_state.velocity[1]

    def clamp(self, pitch_setpoint):
        if pitch_setpoint >= self.max_pitch:
            pitch_setpoint = self.max_pitch
        elif pitch_setpoint <= self.min_pitch:
            pitch_setpoint = self.min_pitch
        return pitch_setpoint

    def setpoint_cmd(self):
        current_time = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = current_time
        dt = current_time - self.last_time
        setpoint_reference = Float64()
        # -Wheel radius due to the velocity being -ve when pitch is forward
        # This can be changed in the URDF
        self.estimated_velocity = -WHEEL_RADIUS * \
            ((self.angular_wheel_L + self.angular_wheel_R) / 2)
        self.velocity_error = self.desired_velocity - self.estimated_velocity


        pitch_setpoint_raw = self.Kv * self.velocity_error + self.Ki * self.integral

        pitch_setpoint = self.clamp(pitch_setpoint_raw)
        
        ##Anti Windup
        if 0 <= dt <= 0.01:
            if pitch_setpoint_raw > self.max_pitch and self.velocity_error < 0:
                self.integral += self.velocity_error * dt
            elif pitch_setpoint_raw < self.min_pitch and self.velocity_error > 0:
                self.integral += self.velocity_error * dt
            else:
                self.integral += self.velocity_error * dt

        pitch_setpoint = self.Kv * self.velocity_error + self.Ki * self.integral
        
        setpoint_reference.data = pitch_setpoint
        self.setpoint_pub.publish(setpoint_reference)

        estimated_velocity = Float64()
        velocity_error = Float64()
        estimated_velocity.data = self.estimated_velocity
        velocity_error.data = self.velocity_error

        self.estimated_velocity_.publish(estimated_velocity)
        self.velocity_error_.publish(velocity_error)
        self.desired_setpoint_error.publish(setpoint_reference)

        self.last_time = current_time



def main():
    rclpy.init()
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
