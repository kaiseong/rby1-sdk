import rby1_sdk as rby
import logging
import numpy as np

from typing import Iterable

def initialize_robot(address, model, power=".*", servo=".*"):
    robot = rby.create_robot(address, model)
    if not robot.connect():
        logging.error(f"Failed to connect robot {address}")
        exit(1)
    if not robot.is_power_on(power):
        if not robot.power_on(power):
            logging.error(f"Failed to turn power ({power}) on")
            exit(1)
    if not robot.is_servo_on(servo):
        if not robot.servo_on(servo):
            logging.error(f"Failed to servo ({servo}) on")
            exit(1)
    if robot.get_control_manager_state().state in [
        rby.ControlManagerState.State.MajorFault,
        rby.ControlManagerState.State.MinorFault,
    ]:
        if not robot.reset_fault_control_manager():
            logging.error(f"Failed to reset control manager")
            exit(1)
    if not robot.enable_control_manager():
        logging.error(f"Failed to enable control manager")
        exit(1)
    return robot


def movej(robot, torso=None, right_arm=None, left_arm=None, minimum_time=0):
    rc = rby.BodyComponentBasedCommandBuilder()
    if torso is not None:
        rc.set_torso_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(torso)
        )
    if right_arm is not None:
        rc.set_right_arm_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(right_arm)
        )
    if left_arm is not None:
        rc.set_left_arm_command(
            rby.JointPositionCommandBuilder()
            .set_minimum_time(minimum_time)
            .set_position(left_arm)
        )

    rv = robot.send_command(
        rby.RobotCommandBuilder().set_command(
            rby.ComponentBasedCommandBuilder().set_body_command(rc)
        ),
        1,
    ).get()

    if rv.finish_code != rby.RobotCommandFeedback.FinishCode.Ok:
        logging.error("Failed to conduct movej.")
        return False

    return True

def rot_y(angle_rad: float) -> np.ndarray:
    """Rotation matrix about Y-axis.

    Args:
        angle_rad: Rotation angle in radians.

    Returns:
        3x3 rotation numpy array.
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle_rad: float) -> np.ndarray:
    """Rotation matrix about Z-axis.

    Args:
        angle_rad: Rotation angle in radians.

    Returns:
        3x3 rotation numpy array.
    """
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def make_transform(r: np.ndarray, t: Iterable[float]) -> np.ndarray:
    """Build a 4x4 homogeneous transform from rotation and translation.

    Args:
        r: 3x3 rotation matrix.
        t: Iterable of 3 floats [x, y, z].

    Returns:
        4x4 homogeneous transform.
    """
    T = np.eye(4)
    T[:3, :3] = r
    T[:3, 3] = np.asarray(t, dtype=float)
    return T

# q=0
# t=0
# q_torso=0
# q_right=0
# q_left=0

# robot =None


# rc = rby.RobotCommandBuilder().set_command(
#     rby.ComponentBasedCommandBuilder().set_body_command(
#         rby.BodyCommandBuilder().set_command(
#             rby.JointPositionCommandBuilder()
#             .set_position(q)
#             .set_minimum_time(t)
#         )
#     )
# )
# rv = robot.send_command(rc, 10).get()


# rc = rby.RobotCommandBuilder().set_command(
#     rby.ComponentBasedCommandBuilder().set_body_command(
#         rby.BodyComponentBasedCommandBuilder().set_right_arm_command(
#             rby.JointPositionCommandBuilder()
#             .set_position(q)
#             .set_minimum_time(t)
#         )
#     )
# )
# rv = robot.send_command(rc, 10).get()

# rc = rby.RobotCommandBuilder().set_command(
#     rby.ComponentBasedCommandBuilder().set_body_command(
#         rby.BodyComponentBasedCommandBuilder()
#         .set_torso_command(
#             rby.JointPositionCommandBuilder().set_position(q_torso)
#         )
#         .set_right_arm_command(
#             rby.JointPositionCommandBuilder().set_position(q_right)
#         )
#         .set_left_arm_command(
#             rby.JointPositionCommandBuilder().set_position(q_left)
#         )
#     )
# )
# rv = robot.send_command(rc, 10).get()



#     rc = rby.BodyComponentBasedCommandBuilder()
#     rc.set_right_arm_command(...)
#     rc.set_left_arm_command(...)

#     stream.send_command(
#         rby.RobotCommandBuilder().set_command(
#             rby.ComponentBasedCommandBuilder().set_body_command(rc)
#         )
#     )


# rc = rby.BodyComponentBasedCommandBuilder()
# rc.set_torso_command(...)
# rc.set_right_arm_command(...)
# rc.set_left_arm_command(...)

# rv = robot.send_command(
#     rby.RobotCommandBuilder().set_command(
#         rby.ComponentBasedCommandBuilder().set_body_command(rc)
#     )
# ).get()

