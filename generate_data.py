import time

import numpy as np
import pybullet as p
import pybullet_data
from tqdm import tqdm

from pyBulletSimRecorder import PyBulletRecorder


def set_robot_state(robot, q: np.ndarray) -> None:
    num_total_joints = p.getNumJoints(robot)
    actuatable_joint_idxs = [
        i
        for i in range(num_total_joints)
        if p.getJointInfo(robot, i)[2] != p.JOINT_FIXED
    ]
    num_actuatable_joints = len(actuatable_joint_idxs)

    assert len(q.shape) == 1, f"q.shape: {q.shape}"
    assert (
        q.shape[0] <= num_actuatable_joints
    ), f"q.shape: {q.shape}, num_actuatable_joints: {num_actuatable_joints}"

    for i, joint_idx in enumerate(actuatable_joint_idxs):
        # q may not contain all the actuatable joints, so we assume that the joints not in q are all 0
        if i < len(q):
            p.resetJointState(robot, joint_idx, q[i])
        else:
            p.resetJointState(robot, joint_idx, 0)


# Setup pyBullet
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setRealTimeSimulation(0)
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf")

recorder = PyBulletRecorder()
# urdf_path = 'assets/power_drill/power_drill.urdf'
# urdf_path = '/juno/u/tylerlum/github_repos/curobo/src/curobo/content/assets/robot/kuka_allegro/model.urdf'
urdf_path = "/juno/u/tylerlum/github_repos/curobo/src/curobo/content/assets/robot/iiwa_allegro_description/iiwa_allegro.urdf"
robot_body_id = p.loadURDF(
    fileName=urdf_path,
    useFixedBase=True,
    # basePosition=(0, 0, 1.4),
    basePosition=(0, 0, 0),
    # baseOrientation=(0.4, 0.3, 0.2, 0.1))
    baseOrientation=(0, 0, 0, 1),
)

drill_urdf_path = "/juno/u/tylerlum/github_repos/pybullet-blender-recorder/assets/power_drill/power_drill.urdf"
drill_body_id = p.loadURDF(
    fileName=drill_urdf_path,
    basePosition=(0, 0, 1.4),
    baseOrientation=(0.4, 0.3, 0.2, 0.1),
)

crackerbox_urdf_path = "/juno/u/tylerlum/github_repos/bidexhands_isaacgymenvs/assets/urdf/reconstructed_crackerbox/model.urdf"
# crackerbox_urdf_path = '/juno/u/tylerlum/github_repos/bidexhands_isaacgymenvs/assets/urdf/ycb/003_cracker_box/model.urdf'
# crackerbox_urdf_path = '/juno/u/tylerlum/github_repos/interactive_robot_visualizer/mesh/crackerbox.urdf'
crackerbox_body_id = p.loadURDF(
    fileName=crackerbox_urdf_path,
    basePosition=(0, 0.5, 1.4),
    baseOrientation=(0.4, 0.3, 0.2, 0.1),
)

# 1. Tell recorder to track a pybullet object
recorder.register_object(robot_body_id, urdf_path)
recorder.register_object(drill_body_id, drill_urdf_path)
recorder.register_object(crackerbox_body_id, crackerbox_urdf_path)

for _ in tqdm(range(500)):
    set_robot_state(robot_body_id, np.random.rand(23))
    p.stepSimulation()
    # 2. Take a snap shot of all registered link poses
    recorder.add_keyframe()
    time.sleep(0.01)

# 3. Dump simulation to a pickle file
recorder.save("demo.pkl")
