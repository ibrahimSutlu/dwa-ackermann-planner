import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive

# Araç parametreleri
WHEELBASE = 0.865  # Dingil mesafesi (örneğin 2.5 metre)

# Ackermann publisher
ackermann_pub = None  

def cmd_vel_callback(msg):
    global ackermann_pub

    # AckermannDriveStamped mesajı oluştur
    ackermann_msg = AckermannDrive()
    ackermann_msg.header.stamp = rospy.Time.now()
    ackermann_msg.header.frame_id = "base_link"

    # Lineer hızı doğrudan atama
    ackermann_msg.drive.speed = msg.linear.x

    # Ackermann direksiyon açısını hesapla
    if msg.linear.x != 0:
        ackermann_msg.drive.steering_angle = math.atan(WHEELBASE * msg.angular.z / msg.linear.x)
    else:
        ackermann_msg.drive.steering_angle = 0.0  # Araç duruyorsa direksiyon açısı 0 olsun

    # Mesajı yayınla
    ackermann_pub.publish(ackermann_msg)
    rospy.loginfo(f"Published Ackermann Drive: Speed={ackermann_msg.drive.speed}, Steering Angle={ackermann_msg.drive.steering_angle}")

def listener():
    global ackermann_pub  

    # ROS node'u başlat
    rospy.init_node('cmd_vel_to_ackermann', anonymous=True)

    # Publisher tanımla
    ackermann_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)

    # /cmd_vel topic'ini dinle
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

    # ROS döngüsünü başlat
    rospy.spin()

if __name__ == '__main__':
    listener()
