## Reference) https://github.com/ros2/teleop_twist_keyboard

import sys, os

import std_msgs.msg
import geometry_msgs.msg
import rclpy

import termios, tty


settings = termios.tcgetattr(sys.stdin)

disable = 0.0

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
---------------------------
Input key layout:
   q    w    e
   a    s    d
   z         c

1~9 : increase/decrease speed step size

q/e : reset speed/turn value
z/c : minimum/maximum servo direction

w/s : increase/decrease only linear speed by step size
a/d : increase/decrease only angular speed by 1

i/o/p : publish command_topic (i=Stop, o=Start, p=Manual)

CTRL-C to quit
"""
keyBindings = {
    'i': (0, 0),
    'o': (1, 0),
    'p': (2, 0),
}

speedBindings = {
    '1': (1, 0),
    '2': (2, 0),
    '3': (3, 0),
    '4': (4, 0),
    '5': (5, 0),
    '6': (6, 0),
    '7': (10, 0),
    '8': (50, 0),
    '9': (100, 0),
}

moveBindings = {
    'w': (1, 0),
    's': (-1, 0),
    'a': (0, -1),
    'd': (0, 1),
    'q': (0, 0),
    'e': (0, 0),
    'z': (0, -90),
    'c': (0, 90),    
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    # sys.stdin.read() returns a string on Linux
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key



def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    rclpy.init()

    node = rclpy.create_node('teleop_twist_keyboard')
    pub = node.create_publisher(geometry_msgs.msg.Twist, 'cmd_vel', 10)
    pub_cm = node.create_publisher(std_msgs.msg.Int8, 'command_topic', 10)

    speed = 0.0
    turn = 0.0
    step = 1.0
    status = 0.0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(settings)
            if key in speedBindings.keys():
                step = speedBindings[key][0]

            elif key in keyBindings.keys():
                #command = keyBindings[key][0]
                command = std_msgs.msg.Int8()
                command.data = keyBindings[key][0]
                pub_cm.publish(command)
                
                if (command.data == 0):
                    status_cm = "Stop"
                elif (command.data == 1):
                    status_cm = "Auto"
                elif (command.data == 2):
                    status_cm = "Manual"
                print("[Publish] command_topic = %d (%s)"%(command.data, status_cm))
                

            elif key in moveBindings.keys():
                speed = speed + moveBindings[key][0]*step
                turn = turn + moveBindings[key][1]*-1
                
                if key == 'q':
                    speed = 0.0
                elif key == 'e':
                    turn = 0.0

                if key == 'z':
                    turn = -90.0
                elif key == 'c':
                    turn = 90.0
                                    
                if (speed < 0):
                    speed = 0.0
                if (turn > 90):
                    turn = 90.0
                elif (turn < -90):
                    turn = -90.0

                print(vels(speed, turn), "\tstep : ", step)
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
                if (key == '\x03'):
                    break
            twist = geometry_msgs.msg.Twist()
            twist.linear.x = speed
            twist.linear.y = disable
            twist.linear.z = disable
            twist.angular.x = disable
            twist.angular.y = disable
            twist.angular.z = turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
