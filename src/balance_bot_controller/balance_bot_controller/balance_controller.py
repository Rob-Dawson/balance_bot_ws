#!/usr/bin/env python3

## TODO Add I
## Add State sensor protection in case messages are paused or removed etc
## Add anti windup


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray, Float64
from tf_transformations import euler_from_quaternion


IN_DEBUG = True

class BalanceController(Node):
    def __init__(self):
        super().__init__("balance_controller")

        if IN_DEBUG:
            self.balance_error_plotjuggler = self.create_publisher(Float64, "/balance_bot_controller/error", 10)


        self.min_effort = -5
        self.max_effort = 5

        self.imu_sub = self.create_subscription(Imu, "imu/data_raw", self.imu_sub_cb, 10)
        self.set_point_sub = self.create_subscription(Float64, "/desired_setpoint", self.setpoint_cb, 10)
        self.balance = self.create_publisher(Float64MultiArray, "/balance_bot_controller/commands", 10)
        self.imu_pitch = None
        self.imu_pitch_rate = None
        
        self.timer = self.create_timer(0.005, self.balance_cmd)
        self.setpoint = 0
        self.declare_parameter("Kp", 0.1)
        self.declare_parameter("Ki", 0.0)
        self.declare_parameter("Kd", 0.02)

        self.Kp = self.get_parameter("Kp").get_parameter_value().double_value
        self.Ki = self.get_parameter("Ki").get_parameter_value().double_value
        self.Kd = self.get_parameter("Kd").get_parameter_value().double_value

        self.add_on_set_parameters_callback(self.event_callback)
   
    def setpoint_cb(self, setpoint:Float64):
        self.setpoint = setpoint.data
        self.get_logger().info(f"{self.setpoint}")

    def event_callback(self, parameter):
        result = SetParametersResult()
        for p in parameter:
            if p.name in ("Kp", "Ki", "Kd"):
                if p.value < 0.0:
                    self.get_logger().info(f"Invalid parameter update: {p.name}: {p.value}")
                    result.successful = False
                    result.reason = "Invalid parameter update"
                    return result

            if p.name == "Kp":
                self.Kp = p.value
            elif p.name == "Ki":
                self.Ki = p.value
            elif p.name == "Kd":
                self.Kd = p.value
        result.successful = True
        return result


    def imu_sub_cb(self, imu:Imu):
        imu_orientation = imu.orientation
        _, self.imu_pitch, _ = euler_from_quaternion([  imu_orientation.x,
                                                    imu_orientation.y,
                                                    imu_orientation.z,
                                                    imu_orientation.w,])
        self.imu_pitch_rate = imu.angular_velocity.y
        

    def balance_cmd(self):
        control = Float64MultiArray()
        # print(self.imu_pitch)
        # if self.imu_pitch is not None:
        #     self.get_logger().info(f"pitch {self.imu_pitch}")

        if self.imu_pitch is None:
            control.data = [0,0]
        elif abs(self.imu_pitch) >= 1.4:
            control.data = [0,0]
           
        else:
            error = self.setpoint - self.imu_pitch
            Kp = self.Kp * error
            Kd = self.Kd * self.imu_pitch_rate

            control_input = Kp - Kd
            if control_input >= self.max_effort:
                control_input = self.max_effort
                self.get_logger().info("Saturated")
            elif control_input <= self.min_effort:
                control_input = self.min_effort
                self.get_logger().info("Saturated")
            # self.get_logger().info(f"Control Input = {control_input}")
            

            control.data = [control_input,control_input]

            if IN_DEBUG:
                balance_error = Float64()
                balance_error.data = error

                balance_pid = Float64MultiArray()
                balance_pid.data = [p, d]

                self.balance_error_plotjuggler.publish(balance_error)
                self.PID_param_plotjuggler.publish(balance_pid)
        self.balance.publish(control)

def main():
    rclpy.init()
    node = BalanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()