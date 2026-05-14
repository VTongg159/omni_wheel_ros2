import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Điều khiển Robot OMNI
W/S : Tiến/Lùi
Q/E : Xoay Trái/Phải 
K : Dừng lại
"""
move_bindings = {

    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'q': (0.0, 1.0),    
    'e': (0.0, -1.0),   
}

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init() 
    node = rclpy.create_node('omni_teleop') 
    pub = node.create_publisher(Twist, 'cmd_vel', 10)
    speed = 0.5 
    turn = 1.0  
    x = 0.0
    th = 0.0
    try:
        print(msg)
        while rclpy.ok():
            key = get_key(settings)
            if key in move_bindings.keys():
                x = move_bindings[key][0] * speed
                th = move_bindings[key][1] * turn
            elif key == ' ' or key == 'k':
                x = 0.0
                th = 0.0
            elif key == '\x03': 
                break
            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(th)
            pub.publish(twist)

    except Exception as e:
        print(e)
    finally:
        twist = Twist() 
        pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()