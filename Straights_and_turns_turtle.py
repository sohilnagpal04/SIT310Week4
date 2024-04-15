import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
import math

class TurtlesimStraightsAndTurns:
    def __init__(self):
        
        # Initialize class variables
        self.last_distance = 0
        self.goal_distance = 0
        self.goal_angle = 0
        self.previous_pose = None
        self.dist_goal_active = False
        self.forward_movement = True
        self.angle_move = False
        self.distance_travelled = 0.0

        # Initialize the node
        rospy.init_node('turtlesim_straights_and_turns_node', anonymous=True)

        # Initialize subscribers  
        rospy.Subscriber("/turtle_dist", Float64,self.distance_callback)
        rospy.Subscriber("/goal_angle", Float64,self.goal_angle_callback)
        rospy.Subscriber("/goal_distance", Float64,self.goal_distance_callback)
        rospy.Subscriber("/turtle1/pose", Pose,self.pose_callback)

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
            # Calculate the distance between current and previous position
            distance_traveled = math.sqrt((msg.x - self.previous_pose.x) ** 2 + (msg.y - self.previous_pose.y) ** 2)
            # Add the distance travelled to the total distance
            self.distance_travelled += distance_traveled
            # print(self.distance_travelled)
        
        # Update previous_pose with current pose for the next iteration
        self.previous_pose = msg

    def distance_callback(self,msg):
        self.goal_distance += msg.data

    def goal_angle_callback(self,msg):
        # Getting data from goal_angle topic
        print(msg)
        self.goal_angle = msg.data
        self.angle_move = True

    def goal_distance_callback(self,msg):
        # Getting data from goal_distance topic
        print(msg)
        self.goal_distance += abs(msg.data)
        self.last_distance = msg.data
        self.dist_goal_active = True

    def timer_callback(self,msg):
        # If some data is recieved in /goal_distance topic
        if self.dist_goal_active:
            twist_msg = Twist()

            # Checking in which direction the robot should move
            if self.last_distance > 0:
                twist_msg.linear.x = 1.0
            else:
                twist_msg.linear.x = -1.0
            self.velocity_publisher.publish(twist_msg)
        # To make the robot stop when the goal distance is reached
        if self.distance_travelled >= self.goal_distance:
            twist_msg = Twist()
            twist_msg.linear.x = 0
            self.velocity_publisher.publish(twist_msg)
            self.dist_goal_active = False
        if self.angle_move:
           # if some data is recieved in /goal_angle topic
            rotate_cmd = Twist()
            clock = False

            # Checking which direction the robot is supposed to rotate
            clock = self.goal_angle > 0
            if not clock:
                rotate_cmd.angular.z = -1.0
            else:
                rotate_cmd.angular.z = 1.0

            # Making the robot stop after rotating a certain angle
            self.velocity_publisher.publish(rotate_cmd)
            rospy.sleep(abs(self.goal_angle) % 36)
            rotate_cmd.angular.z = 0.0
            self.velocity_publisher.publish(rotate_cmd)
            self.angle_move = False


if __name__ == '__main__': 
    try: 
        turtlesim_straights_and_turns_class_instance = TurtlesimStraightsAndTurns()
    except rospy.ROSInterruptException: 
        pass
