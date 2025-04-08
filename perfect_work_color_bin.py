import pybullet as p
import pybullet_data
import time
import numpy as np
import random


def setup_simulation():
    # ✅ Connect to PyBullet with GUI
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # ✅ Load Plane
    plane_id = p.loadURDF("plane.urdf")

    # ✅ Load Robotic Arm (KUKA)
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", basePosition=[0, 0, 0])

    # ✅ Load Balls with Random Colors & Positions
    ball_radius = 0.05
    ball_positions = [[0.5, -0.2, ball_radius], [0.5, 0.2, ball_radius]]
    ball_ids = []
    colors = [[1, 0, 0, 1], [0, 0, 1, 1]]  # Red and Blue Balls
    for pos, color in zip(ball_positions, colors):
        ball_id = p.loadURDF("sphere_small.urdf", pos, globalScaling=ball_radius * 10)
        p.changeVisualShape(ball_id, -1, rgbaColor=color)  # Apply color
        ball_ids.append((ball_id, color))

    # ✅ Load Bins for Sorting
    bin_red = p.loadURDF("tray/traybox.urdf", [1.0, -0.3, 0])
    bin_blue = p.loadURDF("tray/traybox.urdf", [1.0, 0.3, 0])

    # ✅ Color the Bins to Match Ball Colors
    p.changeVisualShape(bin_red, -1, rgbaColor=[1, 0, 0, 1])   # Red
    p.changeVisualShape(bin_blue, -1, rgbaColor=[0, 0, 1, 1])  # Blue

    bins = {"red": bin_red, "blue": bin_blue}

    return robot_id, ball_ids, bins


def move_arm_to_target(robot_id, target_position):
    joint_indices = [0, 1, 2, 3, 4, 5, 6]
    for _ in range(100):
        joint_angles = p.calculateInverseKinematics(robot_id, 6, target_position)
        for i in range(len(joint_indices)):
            p.setJointMotorControl2(robot_id, joint_indices[i], p.POSITION_CONTROL, joint_angles[i])
        p.stepSimulation()
        time.sleep(0.05)


def detect_ball_color(color):
    if color == [1, 0, 0, 1]:  # Red
        return "red"
    elif color == [0, 0, 1, 1]:  # Blue
        return "blue"
    return None


def pick_and_place(robot_id, ball_ids, bins):
    for ball_id, color in ball_ids:
        bin_key = detect_ball_color(color)
        if bin_key is None:
            continue
        bin_id = bins[bin_key]
        ball_pos, _ = p.getBasePositionAndOrientation(ball_id)

        move_arm_to_target(robot_id, [ball_pos[0], ball_pos[1], 0.2])
        move_arm_to_target(robot_id, [ball_pos[0], ball_pos[1], ball_pos[2] + 0.02])

        constraint_id = p.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=6,
            childBodyUniqueId=ball_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0.02],
            childFramePosition=[0, 0, 0]
        )

        move_arm_to_target(robot_id, [ball_pos[0], ball_pos[1], 0.3])
        bin_pos, _ = p.getBasePositionAndOrientation(bin_id)
        move_arm_to_target(robot_id, [bin_pos[0], bin_pos[1], 0.3])
        move_arm_to_target(robot_id, [bin_pos[0], bin_pos[1], 0.1])

        p.removeConstraint(constraint_id)
        move_arm_to_target(robot_id, [bin_pos[0], bin_pos[1], 0.3])


def add_camera_view():
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0.5, 0, 0.5]
    )


def manual_control(robot_id):
    print("🎮 Use W/A/S/D keys to move arm manually!")

    while True:
        keys = p.getKeyboardEvents()

        if ord('w') in keys:
            move_arm_to_target(robot_id, [0.7, 0, 0.3])
        if ord('s') in keys:
            move_arm_to_target(robot_id, [0.4, 0, 0.3])
        if ord('a') in keys:
            move_arm_to_target(robot_id, [0.5, -0.2, 0.3])
        if ord('d') in keys:
            move_arm_to_target(robot_id, [0.5, 0.2, 0.3])

        p.stepSimulation()
        time.sleep(0.1)


def main():
    robot_id, ball_ids, bins = setup_simulation()
    add_camera_view()
    time.sleep(1)

    pick_and_place(robot_id, ball_ids, bins)

    while True:
        p.stepSimulation()
        manual_control(robot_id)     
        time.sleep(0.01)


if __name__ == "__main__":
    main()
