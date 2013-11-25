import rospy
import tf
import std_msgs
import numpy

class Odoloter:
    #Cause the force transducer data are a bastardized array of Floats. Thaaaaanks
    left_index = 2
    right_index = 5

    def __init__(trans_topic,joint_topic):
        self.current_foot_is_right = True
        self.last_foot_is_right = True
        self.left_foot_to_root = numpy.eye(3)
        self.right_foot_to_root = numpy.eye(3)
        rospy.Subscriber(joint_topic, std_msgs.Float32MultiArray,odolotry)
        
    def odolotry(data):
        left_foot_force_z = data.data[Odoloter.left_index]
        right_foot_force_z = data.data[Odoloter.right_index]

        self.current_foot_is_right = (right_foot_force_z > left_foot_force_z)
        if self.current_foot_is_right != self.last_foot_is_right:
            foot_to_root = 


        self.last_foot_is_right = self.current_foot_is_right  
    

def main():
    rospy.init("walking_odometer")    
    o = Odoloter(rospy.param("transform_topic","walking_odom_estimated"),\
                 rospy.param("joint_topic","that_hubo_joints"))

    rospy.spin()

if __name__ == "__main__":
    main()
