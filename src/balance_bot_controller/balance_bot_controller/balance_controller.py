#!/usr/bin/env python3

## TODO Add I and D params
## Change PID params to ros parameters to change them dynamically
## Add State sensor protection in case messages are paused or removed etc
## Add anti windup


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

from std_msgs.msg import Float64MultiArray
class Balance_Controller(Node):
    def __init__(self):
        super().__init__("Balance_Controller")

        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_sub_cb, 10)
        self.balance = self.create_publisher(Float64MultiArray, "/balance_bot_controller/commands", 10)

        self.imu_pitch = None
        
        self.timer = self.create_timer(0.005, self.balance_cmd)
        self.setpoint = 0


        self.Kp = 1
        self.Ki = 0
        self.Kd = 0
    def imu_sub_cb(self, imu:Imu):
        imu_orientation = imu.orientation
        _, self.imu_pitch, _ = euler_from_quaternion([  imu_orientation.x,
                                                    imu_orientation.y,
                                                    imu_orientation.z,
                                                    imu_orientation.w,])

    def balance_cmd(self):
        control = Float64MultiArray()
        if self.imu_pitch is None:
            control.data = [0,0]
            
        else:
            error = self.setpoint - self.imu_pitch
            control_input = self.Kp * error
            control.data = [control_input,control_input]
        self.balance.publish(control)

def main():
    rclpy.init()
    node = Balance_Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()