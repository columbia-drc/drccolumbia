#!/usr/bin/env python
import roslib
roslib.load_manifest('hubo_hose_task_planner')
roslib.load_manifest('python_orocos_kdl')
from openravepy import *
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *
    import numpy
#import rave2realhubo
import sys
#sys.path.append('/home/jweisz/drc/src/hubo_planning_common/src/')
#from base_wheel_turning import *


import pdb
import rospy

import new_utilities as nut
import hubo_planning_common 
from robotCBiRRT import *
from openhubo.comps import *

from hubo_hose_task_planner_msgs.srv import PosePlanningSrv
import tf_conversions.posemath as pm
import actionlib
import hubo_robot_msgs.msg
import roslib.packages
import sensor_msgs.msg 
import os
import roslib.packages

os.environ['OPENRAVE_PLUGINS'] = roslib.packages.get_pkg_dir('drchubo_ikfast') + '/plugins:' + os.environ['OPENRAVE_PLUGINS']



class GraspPlanner( RobotCBiRRT ):
    """
    Contains services that plan pre-grasping and straight line trajectory movements
    
    pre-grasping motions are designed to garuntee that at the end of a planned motion,
    a straight line motion towards a target final goal is possible. 
    """

    def __init__(self, robotfile, scenefile):
        
        sys.argv = []
        RobotCBiRRT.__init__(self, robotfile, scenefile)
        self.env.SetDebugLevel(DebugLevel.Info)
        self.env.LoadProblem(self.prob_manip, self.robot.GetName())
        
        #load ikfast modules
        #for manip in self.robot.GetManipulators():
        #    self.robot.SetActiveManipulator(manip.GetName())
        #    ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot,iktype=IkParameterization.Type.Transform6D)
        #    ikmodel.load()
        #    manip.SetIkSolver(ikmodel.iksolver)
            
            #if manip.GetIkSolver() == None:
            #    ikmodel.autogenerate()
                

        self.footlinknames = ' Body_RAR Body_LAR polyscale 0.5 0.5 0 '
        self.pregraspServer = rospy.Service('pregrasp_planning', PosePlanningSrv, self.preGraspCallback)
        self.straightMoveServer = rospy.Service('move_straight_planning', PosePlanningSrv, self.moveStraightCallback)
        self.showTrajectoryServer = actionlib.SimpleActionServer('show_trajectory',hubo_robot_msgs.msg.JointTrajectoryAction, self.showTrajectory, False)
        self.showTrajectoryServer.start()
        self.plans_dictionary = {}
    
    def GetT0_RefLink(self, frame):
        T0_RefLink = None
        l = self.robot.GetLink(frame)
        #for l in self.robotid.GetLinks():
        #    if(l.GetName() == frame):
        T0_RefLink = l.GetTransform()
            
        return T0_RefLink

    
                                            
    def unpackPoseListToTran(self, pose_msg_list):
        world_transform_list = []
        for pose in pose_msg_list:
            #In future revisions, poses should be stamped and have their own frames
            frame = "Body_TSY"
            pose_as_tran = pm.toMatrix(pm.fromMsg(pose))
            ref_tran = self.GetT0_RefLink(frame)
            final_tran = dot(ref_tran, pose_as_tran)
            world_transform_list.append(final_tran)
        return world_transform_list
            

    def pregraspCBiRRT(self, pregrasp_pose, target_pose, manip_index = ComplexPlanningSrvRequest.RIGHT_ARM_INDEX, willPlayback = True):
        """
        @brief - Find a trajectory to the target pose that then allows straight line
        motion from pregrasp_pose to target_pose ignoring collisions between the
        manipulator and the environment

        @param pregrasp_pose -  The 4x4 transform matrix that we will want
                                to put the end effector at immediately as a 4x4
                                transform matrix

        @param target_pose - The final pose 4x4 transform matrix that we will want
                             to reach in world coordinates.

        @param manip_index - The manipulator identifier, either as a string or an integer

        @returns a hubo_common_msgs/HuboTrajectoryMsg if a valid plan is found
        None otherwise.
        """
        traj = None
        
        #lock environment
        with self.env:
            straight_line_trajectory = None
            #Setup manipulator
            manip = self.setupManipulator(manip_index)

            #Get starting configuration - It may be changed by the straight line
            #planner later
            startjoints = self.robot.GetActiveDOFValues()

            #The hand is allowed to be in collision with the target environment
            #or the world in the final straight line motions to the grasp -- This allows
            #us to push through the hose or other unimportant occlusions
            #and to not worry about having too much padding on the target
            #object to allow us to grasp it. 
            self.setEndEffectorCollisions(manip_index, False)            


            #Get a generator for straight line paths between these two points
            straight_line_generator = self.straightLinePlanner(pregrasp_pose, target_pose, manip_index)
            
        
            #For each viable straight line trajectory, try to plan a motion to the start
            #of that trajectory
            for straight_traj in straight_line_generator:
                #If the current trajectory is invalid, try the next one.
                if straight_traj[1] == None:
                    continue
                
                #The arm motion to the pregrasp pose should not ignore collisions
                self.setEndEffectorCollisions(manip_index, True)            
                
                #Get the joint goal for this the start of this trajectory
                goaljoints = numpy.array(straight_traj[2].GetWaypoint(0))[:7]


                #Compose the planning string to reach that joint goal
                cmdStr = 'RunCBiRRT '
                cmdStr += 'supportlinks 2 '+self.footlinknames+' smoothingitrs '+str(100)+' jointgoals '+str(len(goaljoints))+' '+Serialize1DMatrix(goaljoints) +' jointstarts ' + str(len(startjoints)) + ' ' + Serialize1DMatrix(startjoints)

                #Run the planner
                result = self.prob_cbirrt.SendCommand(cmdStr)

                endEffectorTransform = None

                #If there was a valid result, get the transform from it
                if result != '':
                    endEffectorTransform = mat(self.robot.GetActiveManipulator().GetEndEffectorTransform())
                    
                    '''
                    or_traj,config=create_trajectory(self.robot)           
                    read_trajectory_from_file(or_traj,'cmovetraj.txt')
                    traj = [straight_traj[1], self.orTrajToHuboTraj(or_traj), or_traj]
                    '''
                    break            
                #If we didn't generate a valid plan, try the next straight line trajectory
                #Disable end effector collisions again before calling the generator
                #The prior state will be restored when we release the environment lock, so
                #don't worry about it. 
                self.setEndEffectorCollisions(manip_index, False)
            
                
        return endEffectorTransform
                
            

    def straightLinePlanner(self, start_pose, end_pose, manip_index, step_distance = .003):
        """
        @brief - Returns a generator for straight line trajectories between the start and
        end poses given.

        Assumes that they have the same orientation. Behavior is undefined if they don't.

        @param start_pose - The 4x4 transform describing the start pose for the plan in
                            world coordinates


        @param end_pose - The 4x4 transform describing the end pose for the plan in
                            world coordinates

        @param manip_index - The manipulator id as either an integer or a name

        @param step_distance - The step size in the comps.manipulation2 plugin. Don't
                               change this unless you know what you are doing.

        @returns a generator that yields [starting_joint_configuration, trajectory] pairs
        if valid, [starting_joint_configuration, None] pairs if no valid path is found.
                            
        """
        def posesToMotion(start_pose, end_pose):
            #Get a description of the desired motion as a direction
            #and number of steps to take from the starting pose
            motion = end_pose[:3,3] - start_pose[:3,3]
            motion_len = numpy.linalg.norm(motion)
            steps = abs(motion_len//step_distance)
            direction = motion/motion_len
            return steps, direction



        def planStraightTrajectory(jointConfig, direction, steps):
            """@brief - Run the straight line planner on a joint configuration
            with the given starting direction. May not actually return a valid plan
            even if a final configuration is returned, because it may not move
            far enough. 
            
            @param Returns a final joint pose
            """
            #Set the robot to the starting configuration
            self.robot.SetActiveDOFValues( jointConfig )

            #Construct the planning string
            cmd = 'LiftArmGeneralIK breakoncollision 0 exec 0 minsteps %d maxsteps %d direction %s filename straightline_plan.traj' % (steps, steps, Serialize1DMatrix(direction))
            #Run the planner
            response = self.prob_manip.SendCommand( cmd )
            result = []
            #If we got a valid result, make it into a joint configuration.
            try:
                result = [float(x) for x in response[:-1].split(' ')]
            except:
                pdb.set_trace()
            
            return result


        def getStraightTrajectory(end_pose, joint_config, direction, steps):
            """
            @brief Creates a generator for straight forward motion from the starting
            position defined in joint_config to the end_pose, in the direction and number of steps given.

            @param end_pose - The end pose to plan to. This is used to check that
                              the planner has gone far enough
            @param joint_config - The starting joint configuration to plan from.

            @param direction - The direction the planner will run in

            @param steps - The number of steps the planner will attempt to take.

            Step size is defined in the comps.manipulation2 plugin.

            @returns a HuboTrajectory message if possible, None if no trajectory is found. 
            """
            result = planStraightTrajectory(joint_config, direction, steps)
            if result:
               self.robot.SetActiveDOFValues(result)            
               final_pose = mat(self.robot.GetActiveManipulator().GetEndEffectorTransform())
               end_effector_error = dot(final_pose, linalg.inv(end_pose))
               dist = numpy.linalg.norm(end_effector_error[:3,3])
               if dist > step_distance:
                   return None, None
            else:
                return None, None
            
            
            or_traj,config=create_trajectory(self.robot)
            
            read_trajectory_from_file(or_traj,'straightline_plan.traj')

            hubo_traj = self.orTrajToHuboTraj(or_traj)
            
            return hubo_traj, or_traj




        #Set up the manipulator
        manip = self.setupManipulator(manip_index)

        #Get the goal as a vector and number of steps
        steps, direction = posesToMotion(start_pose, end_pose)

        #find a set of ik solutions for the starting pose
        solutions = manip.FindIKSolutions(start_pose, IkFilterOptions.CheckEnvCollisions)

        #Otherwise, generate trajectories to the next solution in the list. 
        for ind,j in enumerate( solutions ):           
            traj, or_traj = getStraightTrajectory(end_pose, j, direction, steps)
            yield j, traj, or_traj


    def build_openrave_environment(self, environment):
        """
        @brief - FIXME mostly a stub function to load any objects from the use in to
        the planning environment before planning
        """
        for obstacle in environment.fixedObjects:
            obstacles.append(nut.load_parametric_objects(self.env,obstacle,\
                                                           self.robot.GetTransform()))

    def addToPlanDictionary(self, or_traj, hubo_traj):
        key = Serialize1DMatrix(hubo_traj.points[0].positions) + Serialize1DMatrix(hubo_traj.points[-1].positions)
        self.plans_dictionary[key] = [hubo_traj, or_traj]
    

    def getOrPlanFromDictionary(self, hubo_traj):        
        key = Serialize1DMatrix(hubo_traj.points[0].positions) + Serialize1DMatrix(hubo_traj.points[-1].positions)
        return self.plans_dictionary[key][1]
        
    def showTrajectory(self, hubo_traj_msg):
        or_traj = self.getOrPlanFromDictionary(hubo_traj_msg.trajectory)
        demo_traj_name = 'show_trajectory.txt'
        traj_str = or_traj.serialize()
        f = open(demo_traj_name,'w')
        f.write(traj_str)
        f.close()
        self.prob_cbirrt.SendCommand('traj  %s'%(demo_traj_name))
        self.showTrajectoryServer.set_succeeded()
        return 
    

    def preGraspCallback(self, pose_planning_req):
        """@brief - Callback function that wraps the cbirrt pregrasp planner.
        Assumes that the first pose in the TargetPoses array is the pre-grasp pose
        that we will return a plan to, such that there is a straight line trajectory
        from that pose to the final-grasp pose

        @params pose_planning_req - A PosePlannerSrvRequest object.

        @returns - A PosePlannerSrvResponse object with the trajectory, if found

        """
        #Get a response object to fill
        resp = PosePlanningSrv._response_class()

        #load the environment containing any collisions if given
        if pose_planning_req.Environment is not None:
            self.build_openrave_environment(pose_planning_req.Environment)

        #unpack the transforms
        world_tran_list = self.unpackPoseListToTran(pose_planning_req.TargetPoses)
        pregrasp_pose = world_tran_list[0]
        target_pose = world_tran_list[1]
        #pdb.set_trace()

        #Read the manipulator identifier - currently designated as an integer
        manip_index = pose_planning_req.ManipulatorID

        #Generate pregrasp transform
        transform = self.pregraspCBiRRT(pregrasp_pose, target_pose, manip_index)

        #If we have a valid trajectory, pack it in to the response
        if transform != None:            
            resp.PlannedEETransform = pm.fromMatrix(transform).asMessage()
            resp.success = True
        else:
            resp.success = False
            print resp
        return resp

    def moveStraightCallback(self, pose_planning_req):
        """@brief - Callback function that wraps the linear motion planner
        Assumes that the starting pose is packed in the first entry of the
        TargetPoses array and the final pose is in the second entry of the
        TargetPoses array. Also assumes that if the Environment.TargetObject
        member is set, that the caller wants to grasp an actual object, so we
        will ignore collisions of the end effector itself in making this plan. 
        
        

        @params pose_planning_req - A PosePlannerSrvRequest object.

        @returns - A PosePlannerSrvResponse object with the trajectory, if found.

        """

        #Get a response object to fill 
        resp = PosePlanningSrv._response_class()
        
        #unpack the transforms
        world_tran_list = self.unpackPoseListToTran(pose_planning_req.TargetPoses)
        start_pose = world_tran_list[0]
        end_pose = world_tran_list[1]

        #Read the manipulator identifier - currently designated as an integer
        manip_index = pose_planning_req.ManipulatorID

        
        
        #load the environment containing any collisions if given
        target_object = None
        if pose_planning_req.Environment is not None:
            target_object = self.build_openrave_environment(pose_planning_req.Environment)

        #If a target object has been set, ignore end effecto collisions
        if target_object is not None:
            self.setEndEffectorCollisions(manip_index, False)

        #Create a straight line trajectory generator
        straight_trajectory_generator = self.straightLinePlanner(start_pose, end_pose, manip_index)

        #Run it until a valid trajectory is found
        trajectory = None
        for traj in straight_trajectory_generator:
            if traj[1] is not None:
                self.addToPlanDictionary(traj[2], traj[1])
                trajectory = traj[1]
                break
        #if a valid trajectory was found, pack it in to the response
        if trajectory != None:
            resp.PlannedTrajectory = trajectory
            resp.success = True
        else:
            resp.success = False
        return resp
                    
            

    def setEndEffectorCollisions(self, manip_index, enabled = True):
        #disable collisions between the hand and the other objects in the world
        [link.Enable(enabled) for link in self.robot.GetManipulators()[manip_index].GetChildLinks()]
                                     

    def setupManipulator(self, manip_index):
        self.updateJointStates()
        self.setMinimalFingerOpening()
        #Set the currently active manipulator and set the active dofs of the robot correspondingly
        self.robot.SetActiveManipulator(manip_index)
        manip = self.robot.GetActiveManipulator()
        self.robot.SetActiveDOFs(manip.GetArmIndices())
        return manip            

    def setMinimalFingerOpening(self):
        

        finger_names = ['LF1', 'RF1', 'RF2']
        finger_dof_list = []
        current_active_dofs = self.robot.GetActiveDOFIndices()
        for name in finger_names:
            finger_dof_list.append(self.robot.GetJoint(name).GetDOFIndex())
        self.robot.SetActiveDOFs(finger_dof_list)
        dof_values = self.robot.GetActiveDOFValues()
        for ind in xrange(len(dof_values)):
            if dof_values[ind] > -1e-2:
                dof_values[ind] = -1e-2
        
        self.robot.SetActiveDOFValues(dof_values)
        self.robot.SetActiveDOFs(current_active_dofs)
        return
    

    def orTrajToHuboTraj(self, or_traj):
        rospy.logwarn('object_grasp_server::Using internal orTrajToHuboTraj. This is terrible and needs to die\n')
        finger_cmds = {joint_name: self.robot.GetJoint(joint_name).GetValue(0) for joint_name in ['LF1','RF1','RF2']}
        
        return nut.get_trajectory_msg(or_traj, self.robot, 200, finger_cmds)
    
    def updateJointStates(self):
        
        joint_state = rospy.wait_for_message('/drchubo_fullbody_interface/joint_states',sensor_msgs.msg.JointState)
        
        dof_values = zeros(self.robot.GetDOF())
        current_active_dofs = self.robot.GetActiveDOFIndices()
        for name, position in zip(joint_state.name, joint_state.position):
            try:
                ind = self.robot.GetJoint(name).GetDOFIndex()
                dof_values[ind] = position
            except:
                pass
        
        self.robot.SetDOFValues(dof_values)
        self.robot.SetActiveDOFs(current_active_dofs)
        return



def test():
    if rospy.get_name() == '/unnamed':
        rospy.init_node('test_object_grasp_server')
    left_test_tran = numpy.eye(4)
    left_test_tran[0,3] = .2
    left_test_tran[1,3] = .5
    left_test_tran[2,3] = 1
    left_test_tran_2 = left_test_tran.copy()
    left_test_tran_2[1,3] = .6
    
    right_test_tran = numpy.eye(4)
    right_test_tran[0,3] = -.2
    right_test_tran[1,3] = .2
    right_test_tran[2,3] = 1
    
    g = GraspPlanner("/home/jweisz/drc/src/drchubo/drchubo_v3/robots/drchubo_v3.robot.xml","")


    
    return g,g.pregraspCBiRRT(left_test_tran, left_test_tran_2, 0)


    

def main():
    rospy.init_node("grasp_trajectory_planner")
    robot_file = roslib.packages.get_pkg_subdir('drchubo_v3', 'robots') + "/drchubo_v3.robot.xml"
    
    g = GraspPlanner(robot_file,"")
    

    rospy.spin()


if __name__ == "__main__":
    main()
