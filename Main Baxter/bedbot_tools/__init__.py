from copy import deepcopy as dpcpy


def array_to_pose(pose_in, pose_data, ignore=None):
    pose = dpcpy(pose_in)
    if ignore is None:
        ignore = [0] * 7
    pose_data = pose_data + [0]*(7-len(pose_data))
    
    if ignore[0] is 0:
        pose.position.x = pose_data[0]
    if ignore[1] is 0:
        pose.position.y = pose_data[1]
    if ignore[2] is 0:
        pose.position.z = pose_data[2]
    if ignore[3] is 0:
        pose.orientation.x = pose_data[3]
    if ignore[4] is 0:
        pose.orientation.y = pose_data[4]
    if ignore[5] is 0:
        pose.orientation.z = pose_data[5]
    if ignore[6] is 0:
        pose.orientation.w = pose_data[6]
    return pose

def pose_to_array(pose):
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ]
