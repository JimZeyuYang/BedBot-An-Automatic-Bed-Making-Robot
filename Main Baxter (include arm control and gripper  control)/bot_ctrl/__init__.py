from copy import deepcopy as dpcpy
from bedbot_tools import pose_to_array, array_to_pose

def act_move_end_effect_cartision(group, position_data):
    if type(position_data) != list:
        position_data = pose_to_array(position_data)
    cur_pose = group.get_current_pose().pose
    distance = ( \
        (cur_pose.position.x - position_data[0])**2 + \
        (cur_pose.position.y - position_data[1])**2 + \
        (cur_pose.position.z - position_data[2])**2) ** 0.5

    poses = []

    T = int(distance * 10)
    # print("T = {}".format(T))
    for i in range(T):
        ratio = (i+1) / T
        inv_ratio = 1 - ratio

        pose = dpcpy(cur_pose)

        pose.position.x = cur_pose.position.x * inv_ratio + position_data[0] * ratio
        pose.position.y = cur_pose.position.y * inv_ratio + position_data[1] * ratio
        pose.position.z = cur_pose.position.z * inv_ratio + position_data[2] * ratio
        pose.orientation.x = cur_pose.orientation.x * inv_ratio + position_data[3] * ratio
        pose.orientation.y = cur_pose.orientation.y * inv_ratio + position_data[4] * ratio
        pose.orientation.z = cur_pose.orientation.z * inv_ratio + position_data[5] * ratio
        pose.orientation.w = cur_pose.orientation.w * inv_ratio + position_data[6] * ratio

        poses.append(pose)

    (path, _) = group.compute_cartesian_path(poses, 0.01, 0.0)

    try:
        group.execute(path, wait=True)
        group.stop()
    except Exception as e:
        print(e)


def act_move_end_effect(group, position_data):
    if type(position_data) == list:
        pose = group.get_current_pose().pose
        pose = array_to_pose(pose, position_data)
    else:
        pose = dpcpy(position_data)
    group.set_pose_target(pose)

    try:
        group.go(wait=True)
        group.stop()
    except Exception as e:
        print(e)


def act_move_joint(group, joint_data):
    group.set_joint_value_target(joint_data)
    plan = group.plan()

    if not plan.joint_trajectory.points:
        print("[ERROR] No trajectory found")
    else:
        group.go(wait=True)
        group.stop()
