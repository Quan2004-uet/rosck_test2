#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

# Thông báo hướng dẫn sử dụng
msg = """
Control Your 4 Wheel Robot!
---------------------------
Moving around (press once to move continuously):
   w    : Tiến
   s    : Lùi
   a    : Quay trái
   d    : Quay phải
   space: Dừng

Adjust Speed:
   q    : Tăng tốc độ di chuyển và quay
   z    : Giảm tốc độ di chuyển và quay
   e    : Tăng tốc độ quay
   c    : Giảm tốc độ quay

CTRL-C to quit
"""

# Bảng ánh xạ phím điều khiển
moveBindings = {
    'w': (1, 0),    # Tiến
    's': (-1, 0),   # Lùi
    'a': (0, 1),    # Quay trái
    'd': (0, -1),   # Quay phải
}

# Bảng ánh xạ phím điều chỉnh tốc độ
speedBindings = {
    'q': (1.1, 1.1),  # Tăng tốc độ di chuyển và quay
    'z': (0.9, 0.9),  # Giảm tốc độ di chuyển và quay
    'e': (1.0, 1.1),  # Tăng tốc độ quay
    'c': (1.0, 0.9),  # Giảm tốc độ quay
}

def getKey(settings):
    """Đọc phím từ bàn phím mà không cần Enter."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Timeout 0.1 giây
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    """Hiển thị tốc độ hiện tại."""
    return "currently:\tspeed %s m/s\tturn %s rad/s" % (speed, turn)

def clamp(value, min_value, max_value):
    """Giới hạn giá trị trong khoảng cho phép."""
    return max(min_value, min(value, max_value))

def main():
    settings = termios.tcgetattr(sys.stdin)
    
    # Khởi tạo node ROS
    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    # Khởi tạo tốc độ ban đầu
    speed = 0.1  # m/s
    turn = 0.5   # rad/s
    x = 0.0      # Tốc độ tuyến tính (duy trì liên tục)
    th = 0.0     # Tốc độ góc (duy trì liên tục)
    status = 0

    speed_min, speed_max = 0.05, 1.0  # m/s
    turn_min, turn_max = 0.1, 2.0     # rad/s


    rate = rospy.Rate(50)

    try:
        print(msg)
        print(vels(speed, turn))
        while not rospy.is_shutdown():
           
            key = getKey(settings)
            
        
            if key in moveBindings:
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                rospy.loginfo(f"Key pressed: {key}, x={x}, th={th}")
            elif key in speedBindings:
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
          
                speed = clamp(speed, speed_min, speed_max)
                turn = clamp(turn, turn_min, turn_max)
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            elif key == ' ':
                x = 0.0
                th = 0.0
                rospy.loginfo("Key pressed: space, stopping robot")
            elif key == '\x03':  # Ctrl+C
                rospy.loginfo("Exiting...")
                break

            # Tạo và publish message Twist liên tục
            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)
       
            if x != 0 or th != 0:
                rospy.loginfo(f"Published Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

            rate.sleep()

    except Exception as e:
        print(e)

    finally:

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.loginfo("Robot stopped.")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == "__main__":
    main()

    