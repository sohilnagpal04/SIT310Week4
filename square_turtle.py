import rospy
from geometry_msgs.msg import Twist
import math

def move_turtle_square():
    rospy.init_node('turtlesim_square_node', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.loginfo("Turtles are great at drawing squares!")
    rate = rospy.Rate(1)

    speed = 1.0
    angle_speed = 1.5708

    for i in range(10):

        # Move forward
        move_cmd = Twist()
        move_cmd.linear.x = speed
        velocity_publisher.publish(move_cmd)
        rospy.sleep(2.0)

        # Stop
        stop_cmd = Twist()
        velocity_publisher.publish(stop_cmd)
        rospy.sleep(0.1)

        # Rotate by 90 degree
        rotate_cmd = Twist()
        rotate_cmd.angular.z = angle_speed
        velocity_publisher.publish(rotate_cmd)
        rospy.sleep(1.5708)

        # Stop
        velocity_publisher.publish(stop_cmd)
        rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    try:
        move_turtle_square()
    except rospy.ROSInterruptException:
        pass
