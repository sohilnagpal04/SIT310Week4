import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from turtlesim.msg import Pose
import time 
import math

class DistanceReader:
    def __init__(self):
        rospy.init_node('turtlesim_distance_node', anonymous=True)
        rospy.Subscriber("/turtle1/pose", Pose, self.callback)
        self.distance_publisher = rospy.Publisher('/turtle_dist', Float64, queue_size=10)
        self.previous_pose = None  # To store the previous pose of the turtle
        self.total_distance = 0.0  # Variable to accumulate the total distance traveled
        rospy.loginfo("Initialized node!")
        rospy.spin()

    def callback(self, msg):
        rospy.loginfo("Turtle Position: %s %s", msg.x, msg.y)
        
        if self.previous_pose is not None:
            # Calculating the Distance with the previous node and current node
            distance_traveled = math.sqrt((msg.x - self.previous_pose.x) ** 2 + (msg.y - self.previous_pose.y) ** 2)

            # Adding the Distance to the total distance 
            self.total_distance += distance_traveled

            # Publishing the data to the total distance
            self.distance_publisher.publish(Float64(self.total_distance))
          
        self.previous_pose = msg
if __name__ == '__main__': 
    try: 
        distance_reader_class_instance = DistanceReader()
    except rospy.ROSInterruptException: 
        pass
