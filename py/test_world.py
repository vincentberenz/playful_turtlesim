from world import ROS_world
import time,rospy

if __name__ == "__main__":

    world = ROS_world(4)

    world.start()

    print
    print "this will run for 10 seconds"
    print
    time.sleep(10)
    
    world.stop()
