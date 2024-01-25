import time
import argparse
import subprocess
import os
from os.path import join

import numpy as np
import rospy
import rospkg

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from gazebo_simulation import GazeboSimulation
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf
from geometry_msgs.msg import Quaternion

import DWA
import math
import Astar

INIT_POSITION = [-2, 3, 1.57]  # in world frame
GOAL_POSITION = [0, 10]  # relative to the initial position

forward_step = 10

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

def get_dwa_cmd_vel(gazebo_sim, dwa_config, dwa, goal):
    global_curr_pose = gazebo_sim.get_model_state().pose
    global_curr_twist = gazebo_sim.get_model_state().twist
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([global_curr_pose.orientation.x, global_curr_pose.orientation.y, global_curr_pose.orientation.z, global_curr_pose.orientation.w])
    local_vx = math.cos(yaw) * global_curr_twist.linear.x + math.sin(yaw) * global_curr_twist.linear.y
    state = np.array([global_curr_pose.position.x, global_curr_pose.position.y, yaw, local_vx, global_curr_twist.angular.z])
    return DWA.dwa_planning(state, goal, dwa_config, dwa)

def get_dwa_goal(global_path, curr_pos):
    global_path = np.array(global_path)
    curr_pos = np.array(curr_pos)
    dist = np.sum((global_path - curr_pos)**2, axis=1)
    min_idx = np.argmin(dist)
    print("min_idx: ", min_idx)
    print("global_path[min_idx]: ", global_path[min_idx])
    return global_path[min_idx - forward_step] if min_idx - forward_step >= 0 else global_path[0]



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'test BARN navigation challenge')
    parser.add_argument('--world_idx', type=int, default=0)
    
    # TODO: TEST MAP 50, 150, 200
    parser.add_argument('--gui', action="store_true")
    parser.add_argument('--out', type=str, default="out.txt")
    args = parser.parse_args()
    
    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################
    
    os.environ["JACKAL_LASER"] = "1"
    os.environ["JACKAL_LASER_MODEL"] = "ust10"
    os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"
    
    world_name = "BARN/world_%d.world" %(args.world_idx)
    print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" %(world_name))   
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')
    
    launch_file = join(base_path, 'launch', 'gazebo_launch.launch')
    world_name = join(base_path, "worlds", world_name)
    
    gazebo_process = subprocess.Popen([
        'roslaunch',
        launch_file,
        'world_name:=' + world_name,
        'gui:=' + ("true" if args.gui else "false")
    ])
    time.sleep(5)  # sleep to wait until the gazebo being created
    
    rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
    rospy.set_param('/use_sim_time', True)
    
    # GazeboSimulation provides useful interface to communicate with gazebo  
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    
    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
    
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)
    collided = True
    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset() # Reset to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        time.sleep(1)




    ##########################################################################################
    ## 1. Launch your navigation stack
    ## (Customize this block to add your own navigation stack)
    ##########################################################################################
    
    
    # TODO: WRITE YOUR OWN NAVIGATION ALGORITHMS HERE
    # get laser data : data = gazebo_sim.get_laser_scan()
    # publish your final control through the topic /cmd_vel using : gazebo_sim.pub_cmd_vel([v, w])
    # if the global map is needed, read the map files, e.g. /jackal_helper/worlds/BARN/map_files/map_pgm_xxx.pgm
        
    # # DWA example
    # dwa_base_path = rospack.get_path('dwa_planner')
    # launch_file = join(dwa_base_path, 'launch/local_planner.launch')
    # nav_stack_process = subprocess.Popen([
    #     'roslaunch',
    #     launch_file,
    # ])
    
    # Make sure your navigation stack recives a goal of (0, 10, 0), which is 10 meters away
    # along postive y-axis.
    # import actionlib
    # from geometry_msgs.msg import Quaternion
    # from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
    # nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    # mb_goal = MoveBaseGoal()
    # mb_goal.target_pose.header.frame_id = 'odom'
    # mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
    # mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
    # mb_goal.target_pose.pose.position.z = 0
    # mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
    # nav_as.wait_for_server()
    # nav_as.send_goal(mb_goal)
    ##########################################################################################
    ## 2. Start navigation
    ##########################################################################################
    
    curr_time = rospy.get_time()
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)

    map_path = join(base_path, "worlds/BARN/map_files", "map_pgm_%d.pgm" %args.world_idx)
    print("map_path: ", map_path)
    dwa_config = DWA.DwaConfig(map_path)
    dwa = DWA.Dwa(dwa_config)

    astar = Astar.AStar(curr_coor, goal_coor, "euclidean", map_path)
    # plot = plotting.Plotting(s_start, s_goal)
    path, normalized_path, visited = astar.searching()
    
    # check whether the robot started to move
    while compute_distance(init_coor, curr_coor) < 0.1:
        print("havn't started to move yet")
        dwa_goal = np.array([INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]])
        dwa_cmd_vel = get_dwa_cmd_vel(gazebo_sim, dwa_config, dwa, dwa_goal)
        gazebo_sim.pub_cmd_vel(dwa_cmd_vel)

        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        time.sleep(0.01)

    # start navigation, check position, time and collision
    start_time = curr_time
    start_time_cpu = time.time()
    collided = False
    
    while compute_distance(goal_coor, curr_coor) > 1 and not collided and curr_time - start_time < 100:
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)

        # astar = Astar.AStar(curr_coor, goal_coor, "euclidean", map_path)
        # # plot = plotting.Plotting(s_start, s_goal)
        # path, normalized_path, visited = astar.searching()
        # dwa_goal = np.array(normalized_path[-forward_step] if len(normalized_path) >= forward_step else normalized_path[-1])
        dwa_goal = get_dwa_goal(normalized_path, curr_coor)
        print("dwa_goal: ", dwa_goal)
        # dwa_cmd_vel = get_dwa_cmd_vel(gazebo_sim, dwa_config, dwa, goal_coor)
        dwa_cmd_vel = get_dwa_cmd_vel(gazebo_sim, dwa_config, dwa, dwa_goal)
        gazebo_sim.pub_cmd_vel(dwa_cmd_vel)
        print("dwa_cmd_vel: ", dwa_cmd_vel)
        
        print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" %(curr_time - start_time, *curr_coor), end="\r")
        collided = gazebo_sim.get_hard_collision()
        while rospy.get_time() - curr_time < 0.1:
            time.sleep(0.01)


    
    
    ##########################################################################################
    ## 3. Report metrics and generate log
    ##########################################################################################
    
    print(">>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
    success = False
    if collided:
        status = "collided"
    elif curr_time - start_time >= 100:
        status = "timeout"
    else:
        status = "succeeded"
        success = True
    print("Navigation %s with time %.4f (s)" %(status, curr_time - start_time))
    
    path_file_name = join(base_path, "worlds/BARN/path_files", "path_%d.npy" %args.world_idx)
    path_array = np.load(path_file_name)
    path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
    path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
    path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
    path_length = 0
    for p1, p2 in zip(path_array[:-1], path_array[1:]):
        path_length += compute_distance(p1, p2)
    
    # Navigation metric: 1_success *  optimal_time / clip(actual_time, 4 * optimal_time, 8 * optimal_time)
    optimal_time = path_length / 2
    actual_time = curr_time - start_time
    nav_metric = int(success) * optimal_time / np.clip(actual_time, 4 * optimal_time, 8 * optimal_time)
    print("Navigation metric: %.4f" %(nav_metric))
    
    with open(args.out, "a") as f:
        f.write("%d %d %d %d %.4f %.4f\n" %(args.world_idx, success, collided, (curr_time - start_time)>=100, curr_time - start_time, nav_metric))
        
    gazebo_process.terminate()
    time.sleep(1)
    os.system('killall -9 gazebo; killall -9 gzserver; killall -9 gzclient; killall -9 rosmaster; killall -9 roscore; killall -9 python3')
    time.sleep(1)
