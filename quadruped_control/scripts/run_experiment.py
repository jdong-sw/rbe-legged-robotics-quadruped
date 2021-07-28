from __future__ import print_function

import rospy
import actionlib
import sys
import time
from quadruped_control.msg import MoveAction, MoveGoal
from std_msgs.msg import Float64

def ffsmCB(msg: Float64):
    if (not recording): return

    t = time.time() - start
    ffsm = msg.data
    data = f"{t},{ffsm}"
    f.write(data + "\n")
    print(data)

if __name__ == '__main__':
    try:
        # Check arguments
        assert len(sys.argv) > 0, "Missing data filepath argument."
        filepath = sys.argv[1]

        # Initialize ros
        print("Initializing node...")
        rospy.init_node('experiment_node')

        # Start gait controller action client
        print("Connecting to gait controller...")
        client = actionlib.SimpleActionClient('gait_controller', MoveAction)
        client.wait_for_server()

        # Subscribe to ffsm topic
        print("Subscribing to ffsm topic...")
        rospy.Subscriber("/stability/ffsm", Float64, ffsmCB)

        # Open data file
        print("Opening data file...")
        f = open(filepath, "w+")

        # Send gait goal
        print("Starting experiment...")
        goal = MoveGoal()
        goal.distance = 1
        goal.gaitType = 0
        client.send_goal(goal)

        recording = True
        start = time.time()

        # Waits for the server to finish performing the action.
        client.wait_for_result()
        recording = False
        print("Experiment finished.")

        # Close file
        f.close()


    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)