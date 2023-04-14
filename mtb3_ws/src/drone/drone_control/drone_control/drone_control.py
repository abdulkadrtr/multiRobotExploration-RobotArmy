import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.subscription = self.create_subscription(Pose,'drone/gt_pose',self.odom_callback,1)
        self.subscription = self.create_subscription(Pose,'drone/target_pose',self.target,10)
        self.publisher_takeoff = self.create_publisher(Empty, '/drone/takeoff', 10)
        self.publisher_drone_cmd_vel = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        self.subscription
        self.Kp = 0.6
        self.Ki = 0.0
        self.Kd = 0.0
        self.error_x = 0.0
        self.error_y = 0.0
        self.error_z = 0.0
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.integral_z = 0.0
        self.derivative_x = 0.0
        self.derivative_y = 0.0
        self.derivative_z = 0.0
        self.flag = 0

    def odom_callback(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z

    def target(self, msg):
        self.target_x = msg.position.x
        self.target_y = msg.position.y
        self.target_z = msg.position.z
        self.timer = self.create_timer(0.2, self.go_pose)

    def go_pose(self):
        if self.flag == 0:
            self.flag = 1
            msg = Empty()
            self.publisher_takeoff.publish(msg)
            time.sleep(3)
        self.error_x = self.target_x - self.x
        self.error_y = self.target_y - self.y
        self.error_z = self.target_z - self.z
        self.integral_x += self.error_x * 0.1
        self.integral_y += self.error_y * 0.1
        self.integral_z += self.error_z * 0.1
        self.derivative_x = (self.error_x - self.derivative_x) / 0.1
        self.derivative_y = (self.error_y - self.derivative_y) / 0.1
        self.derivative_z = (self.error_z - self.derivative_z) / 0.1
        v_x = self.Kp * self.error_x + self.Ki * self.integral_x + self.Kd * self.derivative_x
        v_y = self.Kp * self.error_y + self.Ki * self.integral_y + self.Kd * self.derivative_y
        v_z = self.Kp * self.error_z + self.Ki * self.integral_z + self.Kd * self.derivative_z
        msg = Twist()
        msg.linear.x = v_x
        msg.linear.y = v_y
        msg.linear.z = v_z
        self.publisher_drone_cmd_vel.publish(msg)
        if self.error_x < 0.3 and self.error_y < 0.3 and self.error_z < 0.3:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = -0.001
            self.publisher_drone_cmd_vel.publish(msg)
            self.timer.cancel()




def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)
    drone_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
