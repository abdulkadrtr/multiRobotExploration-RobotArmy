import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import math

lookahead_distance = 0.15
speed = 0.1

def pure_pursuit(current_x, current_y, current_heading, path,index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index,len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path)-1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi/4
        v = 0.0
    return v,desired_steering_angle,index

class pathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.subscription = self.create_subscription(Float32MultiArray,'/path',self.get_path,10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        print("Path follower node has been started")


    def get_path(self,msg):
        #Rota buraya list olarak geliyor
        self.path = msg.data
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        i = 0
        threading.Thread(target=self.follow_path, args=(i)).start()



    def follow_path(self,i):
        while True:
            twist = Twist()
            twist.linear.x , twist.angular.z,self.i = pure_pursuit(self.x,self.y,self.yaw,self.path,i)
            if(abs(self.x - self.path[-1][0]) < 0.05 and abs(self.y - self.path[-1][1])< 0.05):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                self.subscription.destroy()
                break
            self.publisher.publish(twist)

    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.orientation.z


def main(args=None):
    rclpy.init(args=args)
    path_follower = pathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
