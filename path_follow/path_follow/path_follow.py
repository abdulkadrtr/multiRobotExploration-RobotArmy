import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import  math , time

lookahead_distance = 0.22 #one bakma mesafesi
speed = 0.18 #maksimum hiz

def euler_from_quaternion(x,y,z,w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

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
        self.subscription_path = self.create_subscription(Float32MultiArray,'/path',self.get_path,10)
        self.subscription_odom = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.publisher_visual_path = self.create_publisher(Path, '/visual_path', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        print("Path follower node has been started")


    def get_path(self,msg):
        #Rota dinleme
        print("Path has been received")
        data_list = [msg.data[i] for i in range(len(msg.data))]
        reshaped_data_list = [(data_list[i], data_list[i+1]) for i in range(0, len(data_list), 2)]
        self.path = reshaped_data_list
        threading.Thread(target=self.follow_path).start()


    def follow_path(self):
        twist = Twist()
        path_msg = Path()
        path_msg.header.frame_id = "merge_map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(self.path)):
            pose = PoseStamped()
            pose.pose.position.x = self.path[i][0]
            pose.pose.position.y = self.path[i][1]
            path_msg.poses.append(pose)
        v = 0.0
        w = 0.0
        i = 0
        while True:
            if not hasattr(self, 'x'):
                continue
            v , w ,i = pure_pursuit(self.x,self.y,self.yaw,self.path,i)
            if(abs(self.x - self.path[-1][0]) < 0.15 and abs(self.y - self.path[-1][1])< 0.15):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                print("Path has been followed")
                break
            twist.linear.x = v
            twist.angular.z = w
            self.publisher_visual_path.publish(path_msg)
            self.publisher_cmd_vel.publish(twist)
            time.sleep(0.1)

    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)


def main(args=None):
    rclpy.init(args=args)
    path_follower = pathFollower()
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()