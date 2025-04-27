#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import sys
import tty
import termios

def get_key():
    """Hàm đọc phím nhấn từ terminal mà không cần Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())  
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def clamp(value, min_value, max_value):
    """Giới hạn giá trị trong khoảng cho phép."""
    return max(min_value, min(value, max_value))

def send_position(pub, link1_pos, link2_pos):
    """Gửi lệnh đặt vị trí cho tay máy"""
    traj = JointTrajectory()
    traj.header = Header()
    traj.header.frame_id = "base_link"
    traj.header.stamp = rospy.Time.now()
    traj.joint_names = ['link1_joint', 'link2_joint']
    
    point = JointTrajectoryPoint()
    point.positions = [link1_pos, link2_pos]
    point.velocities = [0.0, 0.0]  
    point.time_from_start = rospy.Duration(0.1) 
    traj.points.append(point)
    
    pub.publish(traj)

def move_arm():

    rospy.init_node('arm_controller_node', anonymous=True)
    pub = rospy.Publisher('/xe_4/arm_controller/command', JointTrajectory, queue_size=10)
    

    rospy.loginfo("Waiting for Gazebo to be ready...")
    rospy.sleep(2) 
    
    link1_pos = 0.0 
    link2_pos = 0.0 
    step = 0.1       # Bước thay đổi mỗi lần nhấn (0.1 rad cho link1, 0.01 m cho link2)

    # Giới hạn khớp 
    link1_min, link1_max = -0.48, 1.57 
    link2_min, link2_max = -0.06, 0.0   

    # Gửi lệnh đặt vị trí (0, 0) ngay khi khởi động
    rospy.loginfo("Moving arm to home pose (0, 0)...")
    for _ in range(20):  # Gửi 20 lần để đảm bảo plugin nhận được
        send_position(pub, 0.0, 0.0)
        rospy.sleep(0.05)

 
    print("Điều khiển tay máy:")
    print("  j: Quay link1_joint sang trái")
    print("  l: Quay link1_joint sang phải")
    print("  i: Tịnh tiến link2_joint lên")
    print("  k: Tịnh tiến link2_joint xuống")
    print("  o: Thoát")

    rate = rospy.Rate(50)  # 50 Hz để gửi lệnh liên tục
    while not rospy.is_shutdown():
        key = get_key()
        
        if key == 'j': 
            link1_pos = clamp(link1_pos - step, link1_min, link1_max)
        elif key == 'l': 
            link1_pos = clamp(link1_pos + step, link1_min, link1_max)
        elif key == 'i': 
            link2_pos = clamp(link2_pos + step * 0.1, link2_min, link2_max) 
        elif key == 'k': 
            link2_pos = clamp(link2_pos - step * 0.1, link2_min, link2_max)  
        elif key == 'o':  # Thoát
            rospy.loginfo("Exiting...")
            send_position(pub, 0.0, 0.0)  
            break
        
        # Gửi lệnh giữ vị trí liên tục
        send_position(pub, link1_pos, link2_pos)
        if key in ['i', 'j', 'k', 'l']:
            rospy.loginfo(f"Sending command: link1_joint={link1_pos:.2f} rad, link2_joint={link2_pos:.2f} m")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException:
        pass