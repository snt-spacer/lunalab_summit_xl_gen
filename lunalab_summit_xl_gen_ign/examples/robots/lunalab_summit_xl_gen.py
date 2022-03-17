from typing import List

MOVE_GROUP_ARM: str = "arm"
MOVE_GROUP_GRIPPER: str = "gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [
    0.2,
    0.2,
    0.2,
]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [
    1.3,
    1.3,
    1.3,
]


def joint_names(prefix: str = "robot_j2s7s300_") -> List[str]:
    return [
        prefix + "joint_1",
        prefix + "joint_2",
        prefix + "joint_3",
        prefix + "joint_4",
        prefix + "joint_5",
        prefix + "joint_6",
        prefix + "joint_7",
    ]


def base_link_name(prefix: str = "robot_j2s7s300_") -> str:
    return prefix + "link_base"


def end_effector_name(prefix: str = "robot_j2s7s300_") -> str:
    return prefix + "end_effector"


def gripper_joint_names(prefix: str = "robot_j2s7s300_") -> List[str]:
    return [
        prefix + "joint_finger_1",
        prefix + "joint_finger_2",
        prefix + "joint_finger_3",
    ]
