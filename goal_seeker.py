# Import Dependencies
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time
import math

class GoalSeeker:
    def __init__(self):
        
        # Initialize class variables
        self.y1 = 0
        self.x1 = 0
        self.y2 = 0
        self.x2 = 0
        self.last_distance = 0
        self.goal_distance = 0
        self.goal_angle = 0
        self.goal = False
        self.previous_pose = None
        self.dist_goal_active = False
        self.forward_movement = True
        self.angle_move = False
        self.distance_travelled = 0.0
        self.theta = 0

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64,self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64,self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64,self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose,self.pose_callback)
        rospy.Subscriber("/goal", Pose,self.goal_callback)

        # Initialize publishers
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

        # Initialize a timer. The timer callback will act as our main function
        timer_period = 0.01
        rospy.Timer(rospy.Duration(timer_period), self.timer_callback)

        # Printing to the terminal, ROS style
        rospy.loginfo("Initalized node!")
        
        # This blocking function call keeps python from exiting until node is stopped
        rospy.spin()

    def pose_callback(self,msg):
        if self.previous_pose is not None:
            self.x1 = msg.x
            self.y1 = msg.y
            self.theta = msg.theta
            # Calculate the distance between current and previous position
            distance_traveled = math.sqrt((msg.x - self.previous_pose.x) ** 2 + (msg.y - self.previous_pose.y) ** 2)
            # Add the distance traveled to the total distance
            self.distance_travelled += distance_traveled
        
        # Update previous_pose with current pose for the next iteration
        self.previous_pose = msg

    def distance_callback(self,msg):
        self.goal_distance += msg.data

    def goal_angle_callback(self,msg):
        self.goal_angle = msg.data
        self.angle_move = True

    def goal_distance_callback(self,msg):
        self.goal_distance += abs(msg.data)
        self.last_distance = msg.data
        self.dist_goal_active = True

    def goal_callback(self,msg):
        self.x2 = msg.x
        self.y2 = msg.y
        self.goal = True

    def timer_callback(self,msg):
        if self.dist_goal_active:
            twist_msg = Twist()
            if self.last_distance > 0:
                twist_msg.linear.x = 1.0
            else:
                twist_msg.linear.x = -1.0
            self.velocity_publisher.publish(twist_msg)
        if self.distance_travelled >= self.goal_distance:
            twist_msg = Twist()
            twist_msg.linear.x = 0
            self.velocity_publisher.publish(twist_msg)
            self.dist_goal_active = False
        if self.angle_move:
            rotate_cmd = Twist()
            print(self.goal_angle > 0)
            clock = False
            clock = self.goal_angle > 0
            if not clock:
                rotate_cmd.angular.z = -1.0
            else:
                rotate_cmd.angular.z = 1.0
            self.velocity_publisher.publish(rotate_cmd)
            rospy.sleep(abs(self.goal_angle))
            rotate_cmd.angular.z = 0.0
            self.velocity_publisher.publish(rotate_cmd)
            self.angle_move = False
          
        if self.goal:
            # Calculating the angle between the current point and goal point.
            angle_radian = math.atan2(self.y2-self.y1, self.x2-self.x1)

            # Calculating the distance between the current point and goal point.
            distance = math.sqrt((self.x2-self.x1)**2 + (self.y2-self.y1)**2)

            # Rotating the turtle until the theta is same as that calculated above
            cmd = Twist()
            cmd.angular.z = 1
            while round(self.theta,1) != round(angle_radian,1):
                self.velocity_publisher.publish(cmd)
            cmd.angular.z = 0.0

            # Moving the turtle until it reach the goal point
            self.velocity_publisher.publish(cmd)
            while round(self.x1,1) != round(self.x2,1) and round(self.y1,1) != round(self.y2,1):
                cmd.linear.x = 1.0
                self.velocity_publisher.publish(cmd)
            cmd.linear.x = 0.0
            self.velocity_publisher.publish(cmd)
            self.goal = False


if __name__ == '__main__': 
    try: 
        turtlesim_straights_and_turns_class_instance = GoalSeeker()
    except rospy.ROSInterruptException: 
        pass
