from supporting_types import Transform

from hsrb_interface.geometry import quaternion, vector3

def gazebo_bottle_poses_callback(data):
    poses = []
    for i in range(len(data.name)):
        ref_frame = data.name[i]
        if "TOGRASP" in ref_frame:
            tf =  Transform(
                    name=data.name[i],
                    position=vector3(data.pose[i].position.x, data.pose[i].position.y, data.pose[i].position.z),
                    quat=quaternion(
                        x=data.pose[i].orientation.x,
                        y=data.pose[i].orientation.y,
                        z=data.pose[i].orientation.z,
                        w=data.pose[i].orientation.w,
                    ),
                )
            poses.append(tf)
    return poses