#!/usr/bin/env python
import sys, select, tty, termios
import rospy
from std_msgs.msg import String

# A key-board driver that listens for keystrokes and publishes them as 
# std-msgs/String on the keys topic 

# this program uses the termios lubrary to capture raw keystrokes. 
# We want to receive the keys on our programs's standard input as soon as they are pressed
# we need to use 2 attribute old_attr and tty.sectbreak()

if __name__ == '__main__':
  key_pub = rospy.Publisher('keys', String, queue_size=1)
  rospy.init_node("keyboard_driver")
  rate = rospy.Rate(100)
  # BEGIN TERMIOS
  old_attr = termios.tcgetattr(sys.stdin)
  tty.setcbreak(sys.stdin.fileno())
  # END TERMIOS

  print "Publishing keystrokes. Press Ctrl-C to exit..."
  while not rospy.is_shutdown():
    if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
      key_pub.publish(sys.stdin.read(1))
    rate.sleep()

  # BEGIN TERMIOS_END
  # put the console back into standard mode before our programs exits
  termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)
  # END TERMIOS_END
# END ALL