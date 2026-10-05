
import math
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive
import numpy as np
from geometry_msgs.msg import Point
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose, Twist, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry,Path
from std_msgs.msg import Int16,String
import time
import scipy.interpolate as si
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2,LaserScan

show_animation = True

gps_x = 0
gps_y = 0
yaw = 0
def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)
    return u, trajectory
def dwa_control2(x, config, goal):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory2(x, dw, config, goal)
    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):

        self.max_speed =0.5  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 30.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 30.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1
        self.dt = 0.2  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.1
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 4.0
        self.robot_stuck_flag_cons = 0.01  # constant to prevent robot stucked
        self.sim_time_samples_ = 2
        self.robot_type = RobotType.rectangle

        self.ob = np.empty((0, 2))  # (0,2) boyutunda boş dizi

        self.robot_radius = 1.0

        self.robot_width = 0.5
        self.robot_length = 0.865

        # self.ob = np.array([            [ 12.0,  0.0071],

   
        #                     ])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):
    """
    motion model
    """
    global gps_x,gps_y,yaw
    
    x[2] = u[1] * dt
    x[0] = gps_x + 0.5
    x[1] = gps_y 
    x[3] = u[0]
    x[4] = u[1]
    # x[2] += u[1] * dt
    # x[0] += u[0] * math.cos(x[2]) * dt
    # x[1] += u[0] * math.sin(x[2]) * dt
    # x[3] = u[0]
    # x[4] = u[1]

    return x
    # return x



def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    x = np.array(x_init)
    trajectory = np.array([x])  # İlk durumu dizi olarak başlat

    for i in range(config.sim_time_samples_):  # sim_time_samples_ kadar iterasyon yap
        x = motion(x, [v, y], config.dt)  # Hareket modelini uygula
        trajectory = np.vstack((trajectory, x))  # Yeni durumu trajectory'ye ekle

    return trajectory  # Oluşturulan güzergahı döndür

def calc_control_and_trajectory2(x, dw, config, goal):
    """
    calculation final input with dynamic window
    """
    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            plt.plot(trajectory, "-r")

            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            
            final_cost = to_goal_cost + speed_cost 

            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:

                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory

def calc_control_and_trajectory(x, dw, config, goal, ob):

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
            trajectory = predict_trajectory(x_init, v, y, config)
            
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)
            
            final_cost = to_goal_cost + speed_cost + ob_cost

            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:

                    best_u[1] = config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):

    
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

    min_r = np.min(r)
    return 1.0 / min_r  # OK


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost



def gps_data(data):
        global gps_x,gps_y,yaw
        orientation_list = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        # self.gps_x = data.pose.pose.position.x-(0.4325*math.cos(self.yaw))
        # self.gps_y = data.pose.pose.position.y-(0.4325*math.sin(self.yaw))
        
        
        gps_x = data.pose.position.x
        gps_y = data.pose.position.y

def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")


def create_obs_list(scan):
        config.ob = np.empty((0, 2))  # (0,2) boyutunda boş dizi

    # Açıların sınırlarını belirleyin
        min_angle = math.radians(-20)  # -10 dereceyi radyan cinsine çevir
        max_angle = math.radians(20)   # 10 dereceyi radyan cinsine çevir

        # Mesafe eşiği (5m) belirleyin
        max_distance = 5.0

        angle = scan.angle_min
        # LaserScan'deki her mesafeyi kontrol et
        for i in range(len(scan.ranges)):
            r = scan.ranges[i]

            # Geçerli açı aralıkta mı?
            if angle < min_angle or angle > max_angle:
                angle += scan.angle_increment
                continue
            
            # Mesafe eşiği kontrolü (maksimum 5m)
            if r < scan.range_min or scan.range_max < r or r > max_distance:
                angle += scan.angle_increment
                continue
            
            # Engel pozisyonunu hesapla
            pose = np.array([[r * math.cos(angle) + gps_x, r * math.sin(angle) + gps_y]])

            num_fake_obstacles = 3  # X ekseninde kaç tane ekstra engel eklenecek
            spacing = 0.3  # Her bir hayali engel arasındaki mesafe (metre cinsinden)

            for i in range(-num_fake_obstacles, num_fake_obstacles + 1):
                fake_x = pose[0, 0] + i * spacing  # X ekseninde kaydırılmış engel
                fake_y = pose[0, 1]  # Y ekseni sabit kalır
                fake_pose = np.array([[fake_x, fake_y]])
                config.ob = np.vstack((config.ob, fake_pose))
            # config.ob = np.vstack((config.ob, pose.reshape(1, -1)))

            angle += scan.angle_increment

def main(gx=17.0, gy=0.0 , robot_type=RobotType.rectangle):
    rospy.init_node('karar_algoritmasi', anonymous=True)
    global gps_x,gps_y,yaw

    rospy.Subscriber('/slam_out_pose', PoseStamped, gps_data)
    control_pub = rospy.Publisher('ackermann_cmd', AckermannDrive, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, create_obs_list)

    x = np.array([0, 0, 0, 0.0, 0.0])

    goal = np.array([gx, gy])

    ob2 = np.empty((0, 2))  # (0,2) boyutunda boş dizi
    config.robot_type = robot_type
   
    trajectory = np.array(x)
    static_obstacles = set()  # Engelleri saklamak için bir küme oluştur
    deneme = 0
    bir_kere = 0
    while not rospy.is_shutdown():
        ob = config.ob  # Engellerin listesi

        if bir_kere == 10 :
            for obstacle in ob:
                static_obstacles.add(tuple(obstacle))  # Engel koordinatlarını kaydet
                
            deneme = 1

            bir_kere = 11

        elif ob.size > 0 :
            if len(static_obstacles) > 0 :

                bir_kere = 12

            else :

                bir_kere = 10


        elif bir_kere == 11 :
            bir_kere = 10
            
        if deneme == 1:

            u, predicted_trajectory = dwa_control(x, config, goal,  np.array(list(static_obstacles))  )
            x = motion(x, u, config.dt)  



            control_pub.publish(u[1], 0, u[0], 0, 0)
            trajectory = np.vstack((trajectory, x)) 

            if show_animation:
                plt.cla()
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                # plt.plot(u[1], "-g")
                plt.plot(trajectory,"r")

                plt.plot(x[0], x[1], "xy")
                plt.plot(goal[0], goal[1], "xb")
                plt.plot(ob[:, 0], ob[:, 1], "ok")
                plot_robot(x[0], x[1], x[2], config)
                # plot_arrow(x[0], x[1], x[2])
                plt.axis("equal")
                plt.grid(True)
                plt.pause(0.0001)
            

            dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
            if dist_to_goal <= config.robot_radius:
                print("Goal!!")
                break
        else:
            
            u, predicted_trajectory = dwa_control2(x, config, goal)
            x = motion(x, u, config.dt)  
            control_pub.publish(u[1], 0, u[0], 0, 0)
            trajectory = np.vstack((trajectory, x))  # Trajectory'yi güncelle

            # Hedefe yakınsa bitir
            dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
            if dist_to_goal <= config.robot_radius:
                print("Goal!!")
                break

    rospy.spin()

if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
