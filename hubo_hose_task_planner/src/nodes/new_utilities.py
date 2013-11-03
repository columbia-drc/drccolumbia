import time
import numpy as np
import lxml.etree
import lxml.builder
import urllib
import sys
import re
import os.path
import pdb
from numpy import pi
import roslib.packages;PKG="openhubo_planning_server"
import rospy
from tf import TransformerROS
from tf.transformations import quaternion_from_euler,euler_from_quaternion
#from openhubo import planning
import openhubo as oh
from openhubo import hands
from openhubo.trajectory import *
from openravepy import RaveCreateTrajectory,planningutils,CollisionReport
from openravepy import KinBody, RaveCreateKinBody, TriMesh

#msgs
from hubo_robot_msgs.msg import *
from hubo_vision_common.msg import *

# DRC Hubo
lArmDOFs = [1,2,3,4,5,6,7]
rArmDOFs = [19,20,21,22,23,24,25]
lHandDOFs = [8,9,10]
lHandVels = [1,1,-1]
rHandDOFs = [26,27,28]
rHandVels = [1,1,-1]

# Hubo+ ?
#lArmDOFs = [14,16,18,20,22,24]
#rArmDOFs = [13,15,17,19,21,23]
#lHandDOFs = [42,43,44, 45,46,47, 48,49,50, 51,52,53, 54,55,56]
#rHandDOFs = [27,28,29, 30,31,32, 33,34,35, 36,37,38, 39,40,41]

#Goes from the openhubo index to the hubo ach index...
openhubo_to_hubo_ach = {0:0,1:26,2:27,3:28,4:29,5:30,6:31,7:11,8:12,9:13,10:14,11:15,12:17,13:16,14:1,15:2,16:3,17:33,18:32,19:4,20:5,21:6,22:7,23:8,24:10,25:9,26:19,27:20,28:21,29:22,30:23,31:24,32:37}

hubo_joint_names={
0:"WST",
1:"NKY",
2:"NK1",
3:"NK2",
4:"LSP",
5:"LSR",
6:"LSY",
7:"LEB",
8:"LWY",
9:"LWR",
10:"LWP",
11:"RSP",
12:"RSR",
13:"RSY",
14:"REB",
15:"RWY",
16:"RWR",
17:"RWP",
19:"LHY",
20:"LHR",
21:"LHP",
22:"LKN",
23:"LAP",
24:"LAR",
26:"RHY",
27:"RHR",
28:"RHP",
29:"RKN",
30:"RAP",
31:"RAR",
32:"RF1",
33:"RF2",
34:"RF3",
35:"RF4",
36:"RF5",
37:"LF1",
38:"LF2",
39:"LF3",
40:"LF4",
41:"LF5"
}


def loadOpenRAVETrajFromFile(robot, filename):
    f=open(filename,'r')
    trajstring=f.read()
    f.close()

    traj=RaveCreateTrajectory(robot.GetEnv(),'')
    traj.deserialize(trajstring)

    planningutils.RetimeActiveDOFTrajectory(traj,robot,True)

    return traj

def sampleTraj(traj, robot, jointIndices, executeTime, dt=0.01):
    #env = robot.GetEnv()
    spec=traj.GetConfigurationSpecification() # get the configuration specification of the trajrectory

    print 'total simulation time: %s'%(traj.GetDuration())
    numSample = executeTime / dt
    simDt = traj.GetDuration() / numSample
    newTraj = list()

    for i in range(int(numSample+1)):
        simTime = simDt * i
        #with env: # have to lock environment since accessing robot
        trajdata=traj.Sample(simTime)
        values=spec.ExtractJointValues(trajdata,robot,range(robot.GetDOF()),0)
        robot.SetDOFValues(values)
        newTraj.append(values[jointIndices])
        time.sleep(dt)

    return newTraj

#functions reverses trajectory so that they can be played backwards
def reverse_trajectory(source, destination):
    f = open(destination, 'w')
    for line in reversed(open(source).readlines()):
        string =  line.rstrip()+"\n"
        f.write(string)
    f.close()

#generates dynamic environment file
#takes object_name and position
def generateEnvironment(object_name, x, y, z):
    root= lxml.etree.Element("environment")
    lxml.etree.SubElement(root, "camtrans").text = "1.2 0.56 1.58"
    lxml.etree.SubElement(root, "camrotationaxis").text = "-0.41 -0.784 0.46 146.11"
    lxml.etree.SubElement(root, "camfocal").text = "1.492339"

    kinbody = lxml.etree.SubElement(root, "kinbody", name= "floor")
    body = lxml.etree.SubElement(kinbody, "body", type = "static")
    lxml.etree.SubElement(body, "translation").text = "0 0 -.5"
    geom = lxml.etree.SubElement(body, "geom", type = "box")
    lxml.etree.SubElement(geom, "extents").text = "5 5 .5"
    lxml.etree.SubElement(geom, "diffusecolor").text = ".81 .4 .4"
    lxml.etree.SubElement(geom, "ambientcolor").text = ".4 .5 .6"

    #generates object xml file, including position of object
    kinbody1 = lxml.etree.SubElement(root, "kinbody", name= object_name, file="../kinbody/"+object_name+".kinbody.xml")
    lxml.etree.SubElement(kinbody1, "translation").text= ('%s %s %s' %( x, y, z))
    #".23524 .1 1.27435"

    #kinbody2 = lxml.etree.SubElement(root, "kinbody", name= "hose", file="../kinbody/hose.kinbody.xml")
    #lxml.etree.SubElement(kinbody2, "translation").text= ".278 .161 1.138"

    print lxml.etree.tostring(root, pretty_print=True)
    try:
        f = open("scenes/hoseexp1.env.xml",'w')
        f.write(lxml.etree.tostring(root, pretty_print=True))
        f.close()
        print("Environment file successfully generated")
    except:
        print("Error: cannot write to file")


    return

##  A collection of post-processing calls
#   One should always call with a full finger states object
def process_trajectory(robot,source,hz,finger_cmds = {'LF1': 0 , 'RF1':0,'RF2':0}):
    
    #check if things worked out
    if not os.path.isfile('%s'%source):
        raise IOError('Trajectory file not found. Bad things have happened')

    #Create the trajectory
    traj,config=create_trajectory(robot)

    read_trajectory_from_file(traj,'%s'%source)

    return process_basic_trajectory(robot,traj,source,hz,finger_cmds=finger_cmds)

## Simplified post processing
def process_basic_trajectory(robot,traj,source,hz,finger_cmds):
    dest = '%s.hubo.traj'%source
    write_hubo_traj(traj,robot,1.0/hz,filename=dest,finger_cmds=finger_cmds)
    reverse_trajectory('%s.hubo.traj'%source,'%s.hubo.reverse.traj'%source)
    return get_trajectory_msg(traj,robot,hz,finger_cmds=finger_cmds)

##  Gets a trajectory_msg to send out
#   This NEEDS better unification with write_hubo_trajectory
#   I should talk to Rob eventually
def get_trajectory_msg(traj,robot,hz,finger_cmds={}):

    hubo_traj = JointTrajectory()

    hubo_traj.header.stamp = rospy.Time.now()

    for key, value in hubo_joint_names.iteritems():
        hubo_traj.joint_names.append(value)

    #Sampling the godamn trajectory
    dt=1.0/hz
    sampler=make_joint_value_sampler(robot,traj, traj.GetConfigurationSpecification())
    num_points = int(traj.GetDuration()/dt)
    
    
    ha_ind_dict = mapping.ha_ind_from_oh_ind(robot)
    
    used_joint_indices = [int(ind) for ind in traj.GetConfigurationSpecification().GetGroupFromName("joint_values").name.split()[2:]] #traj.GetConfigurationSpecification().ExtractUsedIndices(robot)
    num_joints = robot.GetDOF()
    used_joint_dict = {}

    #FIXME this really should be a dictionary comprehension, but that didn't seem to work 
    used_joint_dict = dict([[joint_ind,ha_ind_dict[joint_ind]] for joint_ind in used_joint_indices ])


    
    points = []
    for t in np.arange(0,traj.GetDuration(),dt):
        vals=sampler(t)
        current_point = JointTrajectoryPoint()
        current_point.time_from_start = rospy.Duration(t)
        current_point.positions = np.zeros(max(openhubo_to_hubo_ach.values())+1)
        
        #Move the points from open hubo arrangement to hubo ach arrangement
        for oh_index,ha_index in used_joint_dict.iteritems():
            #openhubo_to_hubo_ach.iteritems():
            ha_name = hubo_joint_names[ha_index]
            #if ha_name in finger_cmds:
            #    current_point.positions[ha_index] = finger_cmds[ha_name]
            #else:
            current_point.positions[ha_index] = vals[oh_index]

        points.append(current_point)

    num_joints = len(points[0].positions)

    #First pass is done, we have positions. Now run again and get velocities and accelerations
    for i in xrange(0,num_points+1):
        points[i].velocities = np.zeros(max(openhubo_to_hubo_ach.values())+1)
        points[i].accelerations = np.zeros(max(openhubo_to_hubo_ach.values())+1)
        for j in xrange(0,num_joints):
            #Simple forward differences...oh physics
            #When we run out of forward points, we put 0
            if i == num_points-1:
                points[i].velocities[j] = (points[i+1].positions[j]-points[i].positions[j])/dt
                points[i].accelerations[j] = 0
            elif i < num_points-1:
                points[i].velocities[j] = (points[i+1].positions[j]-points[i].positions[j])/dt
                points[i].accelerations[j] = (points[i+2].positions[j]-2*points[i+1].positions[j]+points[i].positions[j])/(dt*dt)
    hubo_traj.points = points
    hubo_traj.compliance.joint_names = []
    return hubo_traj


## I am really gonna regret this function
#  Just the left hand for now --- realized it would just be blocked by rob's
#  code anyway
def set_traj_fingers(traj,isOpen=False,man_index=0):
    #Go through each waypoint and set the finger to close or open
    num_waypoints = traj.GetNumWaypoints()
    print num_waypoints
    for index in xrange(0,num_waypoints):
        wpoint = traj.GetWaypoint(index)
        print wpoint
        if isOpen:
            wpoint.positions[33] = -.1
        else:
            wpoint.positions[33] = .1
        traj.Insert(index,wpoint,True)

##Probably not necessary as its own function
# but I suspect later params will need interpretation
def form_param_dict(target_object):
    return dict(zip(target_object.parameterNames,target_object.parameters))

## Big old switch/builder function for parameterized objects in openRAVE
def load_parametric_object(env,parametric_object,robot_tf,isTarget=False):
    object_parameters = form_param_dict(parametric_object)
    with env:
        to_body = None
        #Create the kinematic body
        target_object = None
        if(parametric_object.objectID==ParametricObjectMsg.OBJECT_CLASS_TORUS):
            #Set up geometry
            '''tmesh = env.ReadTrimeshURI(roslib.packages.get_pkg_dir(PKG) + '/openhubo/kinbody/models/hydrant.wrl')
            target_object.InitFromTrimesh(tmesh,True)
            tgeom = target_object.GetLink('base').GeometryInfo()
            tgeom._vRenderScale = tgeom._vCollisionScale = [.001,.001,.001]
            tgeom._type = KinBody.Link.GeomType.Trimesh
            target_object.InitFromGeometries([tgeom])'''
            target_object = env.GetKinBody('hydrant_vertical')
            
        else:
            target_object = RaveCreateKinBody(env,'')
            if(parametric_object.objectID==ParametricObjectMsg.OBJECT_CLASS_HOSE):
                #Set up geometry
                to_body = KinBody.Link.GeometryInfo()
                to_body._type = KinBody.Link.GeomType.Cylinder
                to_body._vGeomData = [in_to_m(1.25),in_to_m(3.5)]
                to_body._vDiffuseColor = [0,1,0]

            elif(parametric_object.objectID==ParametricObjectMsg.OBJECT_CLASS_BOX):
                #Set up geometry
                to_body = KinBody.Link.GeometryInfo()
                to_body._type = KinBody.Link.GeomType.Box
                to_body._vGeomData = [object_parameters['x'],object_parameters['y'],object_parameters['z']]
                to_body._vDiffuseColor = [1,0,0]

            elif(parametric_object.objectID==ParametricObjectMsg.OBJECT_CLASS_CYLINDER):
                #Set up geometry
                to_body = KinBody.Link.GeometryInfo()
                to_body._type = KinBody.Link.GeomType.Cylinder
                to_body._vGeomData = [object_parameters['r'],object_parameters['h']]
                to_body._vDiffuseColor = [1,0,0]

            target_object.InitFromGeometries([to_body])

        #target_object.Enable(False)
        target_object.SetName(parametric_object.name)
        target_object.SetTransform(get_local_pose_transform(parametric_object.pose,robot_tf))

        #Add and update the bodies
        env.Add(target_object,True)
        env.UpdatePublishedBodies()
        return target_object

def get_local_pose_transform(pose,local_frame):
    return np.dot(local_frame,get_pose_transform(pose))

def get_pose_transform(pose):
    #Get the transform/rotation and add the object
    tfROS = TransformerROS()
    tran = (pose.position.x,pose.position.y,pose.position.z)
    rot = (pose.orientation.x,pose.orientation.y,
           pose.orientation.z,pose.orientation.w)
    return tfROS.fromTranslationRotation(tran,rot)

def get_local_name(rel_path,filename):
    #Name determined by object, frequency and system time
    return "%s/traj/%s"%(rel_path,filename)

## Inches to meters if that wasn't obvious
#  Functions are more explicit than sheer math
def in_to_m(inches):
    return 0.0254*inches

def populate_quaternion(orientation,xangle,yangle,zangle):
    for k,v in zip(['w','x','y','z'],\
                   quaternion_from_euler(np.radians(xangle),np.radians(yangle),np.radians(zangle))):
        setattr(orientation, k, v) 
    
## Sort of a hack function to make this matrix. It approximates a function in openHubo
#  I haven't had time to investigate if the openHubo function is working t.t
#  -----There's time now! This displaces the graper so as to avoid finger collision
#  -----Most definitely needs to be handled on an object by oject basis in the future
def getEEMatrix(target_object,man_index):
  link = target_object.GetLink('base')
  if link == None:
    link = target_object.GetLink('body') #Damn openrave uses divergent naming conventions

  hand_reversal = [[-1, 0, 0, 0],
                  [0, -1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0,  0, 1]]

  to_link_geom = link.GetGeometries()[0]

  if to_link_geom.GetType() == KinBody.Link.GeomType.Cylinder:
      T_we = array([[ 1, 0,  0, 0.0],
                   [ 0, 1,  0, to_link_geom.GetCylinderRadius()],
                   [ 0, 0,  1,  -0.05],
                   [ 0, 0,  0,  1.0000]])
      if(man_index == 1):
        T_we = np.dot(T_we,hand_reversal)
      return T_we
  elif to_link_geom.GetType() == KinBody.Link.GeomType.Trimesh:
      T_we = array([[ 1, 0,  0, 0.0],
                   [ 0, 1,  0, .032],
                   [ 0, 0,  1,  -0.05],
                   [ 0, 0,  0,  1.0000]])
      if(man_index == 1):
        T_we = np.dot(T_we,hand_reversal)
      return T_we

