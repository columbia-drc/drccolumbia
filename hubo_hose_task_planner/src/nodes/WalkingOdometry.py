import rospy
import tf
from tf.posemath import PoseMath
import std_msgs

class Odoloter:
    #Cause the force transducer data are a bastardized array of Floats. Thaaaaanks
    left_index = 2
    right_index = 5

    def __init__(trans_topic,joint_topic):
        self.current_foot_is_right = False
        self.last_foot_is_right = False

        #Load the desired topic name from rosparam
        self.trans_topic = trans_topic

        #We start with the left foot
        self.foot_from_root = PoseMath.fromEuler(0,0,0,0,0,0)

        #Build a tf setup
        self.odo_tf = tf.TransformBroadcaster()
        self.robot_tf = tf.TransformListener()

        #Hook the transducer topic into an odometry function
        rospy.Subscriber(joint_topic, std_msgs.Float32MultiArray,odolotry)
        
    def odolotry(data):
        #Get our left and right force readings
        left_foot_force_z = data.data[Odoloter.left_index]
        right_foot_force_z = data.data[Odoloter.right_index]

        #See if we have right foot contact
        self.current_foot_is_right = (right_foot_force_z > left_foot_force_z)
        if self.current_foot_is_right != self.last_foot_is_right:
            #Get our transform between feet
            foot_from_foot = PoseMath.fromTf(robot_tf.lookupTransform('/rightFoot', '/leftFoot', rospy.Time(0)))
            if not self.current_foot_is_right:
                foot_from_foot = ~foot_from_foot

            #Transform our odometry forward
            self.foot_from_root = foot_from_foot*self.foot_from_root 

        #Get the transform from the current foot to the torso 
        if self.current_foot_is_right:
            tsr_from_foot = PoseMath.fromTf(robot_tf.lookupTransform('/TSR', '/rightFoot', rospy.Time(0)))
        else:
            tsr_from_foot = PoseMath.fromTf(robot_tf.lookupTransform('/TSR','/leftFoot', rospy.Time(0)))

        #Build the transform from root to the torso
        tsr_from_root = tsr_from_foot*self.foot_from_root
        self.odo_tf.sendTransform(tsr_from_root,rospy.Time.now(),'/TSR',self.trans_topic)

        #Store the change in support foot
        self.last_foot_is_right = self.current_foot_is_right  
    

def main():
    rospy.init("walking_odometer")    
    o = Odoloter(rospy.param("transform_topic","walking_odom_estimated"),\
                 rospy.param("joint_topic","that_hubo_joints"))

    rospy.spin()

if __name__ == "__main__":
    main()
