import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Global değişkenler
linear_velocity = {"x": 0, "y": 0, "z": 0}
angular_velocity = {"x": 0, "y": 0, "z": 0}

# Zaman değişkeni
last_time = None  

# Publisher tanımla
odom_pub = None  

def imu_callback(msg):
    global linear_velocity, angular_velocity, last_time, odom_pub

    # İlk veri geldiğinde last_time'i başlat
    if last_time is None:
        last_time = rospy.Time.now()
        return  

    # Zaman farkını hesapla
    current_time = rospy.Time.now()
    delta_time = (current_time - last_time).to_sec()
    last_time = current_time

    # Doğrusal ivme (linear acceleration) bileşenleri
    ax = msg.linear_acceleration.x
    ay = msg.linear_acceleration.y
    az = msg.linear_acceleration.z

    # Açısal hız (angular velocity) bileşenleri
    wx = msg.angular_velocity.x
    wy = msg.angular_velocity.y
    wz = msg.angular_velocity.z

    # Doğrusal hızları hesapla (ivme * zaman)
    linear_velocity["x"] = ax *delta_time
    linear_velocity["y"] = ay * delta_time
    linear_velocity["z"] = az  * delta_time
        
    # Açısal hızları güncelle
    angular_velocity["x"] = wx
    angular_velocity["y"] = wy
    angular_velocity["z"] = wz

    # Odometry mesajı oluştur
    odom_msg = Odometry()

    # Header bilgileri
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_footprint"

    # Twist (linear ve angular hızlar)
    odom_msg.twist.twist.linear.x = linear_velocity["x"]
    odom_msg.twist.twist.linear.y = linear_velocity["y"]
    odom_msg.twist.twist.linear.z = linear_velocity["z"]
    odom_msg.twist.twist.angular.x = angular_velocity["x"]
    odom_msg.twist.twist.angular.y = angular_velocity["y"]
    odom_msg.twist.twist.angular.z = angular_velocity["z"]

    # Twist kovaryans matrisi (örnek bir kovaryans değeri atanmıştır)
    odom_msg.twist.covariance = [0.0] * 36  

    # Odometry mesajını yayınla
    odom_pub.publish(odom_msg)

    # Hesaplanan değerleri yazdır
    rospy.loginfo(f"Published Odometry Message: {odom_msg}")

def listener():
    global last_time, odom_pub  

    # ROS node'u başlat
    rospy.init_node('imu_listener', anonymous=True)
    
    # Odometry publisher oluştur
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    # İlk zaman damgasını al
    last_time = rospy.Time.now()

    # 'imu/data' topic'ini dinle (veri geldiğinde imu_callback fonksiyonunu çağır)
    rospy.Subscriber('/zed2/imu/data', Imu, imu_callback)

    # ROS döngüsünü başlat
    rospy.spin()

if __name__ == '__main__':
    listener()
