"""Verify local dependencies and SDK messages; does not initialize ROS."""
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))


def check():
    import numpy
    import scipy
    import pinocchio
    import PIL
    import rclpy
    from aimdk_msgs.msg import JointCommandArray,JointCommand,JointStateArray,JointState,HandCommandArray,HandCommand,HandType,HandStateArray
    from sensor_msgs.msg import CompressedImage,Image,Imu
    from x2_kinematics.aimdk_bridge.protocol import verify_sdk_types
    result=verify_sdk_types(JointCommandArray,JointCommand,JointStateArray,JointState)
    from rclpy.serialization import serialize_message,deserialize_message
    for message_type in (JointCommandArray,JointStateArray,HandCommandArray,HandCommand,HandType,HandStateArray,CompressedImage,Image,Imu):
        deserialize_message(serialize_message(message_type()),message_type)
    return dict(python=sys.version.split()[0],numpy=numpy.__version__,scipy=scipy.__version__,
        pinocchio=pinocchio.__version__,pillow=PIL.__version__,sdk_types=result,
        message_serialization=True,ros_initialized=rclpy.ok(),robot_commands_sent=0)


if __name__=='__main__':print(json.dumps(check(),ensure_ascii=False,indent=2))
