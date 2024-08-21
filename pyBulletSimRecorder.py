import pickle
from pathlib import Path
from typing import Dict, List

import numpy as np
import pybullet as p
from transforms3d.affines import decompose
from transforms3d.quaternions import mat2quat
from urdfpy import URDF

from custom_types import Frame, Trajectory


class PyBulletRecorder:
    class LinkTracker:
        def __init__(
            self,
            name: str,
            body_id: int,
            link_id: int,
            link_origin: np.ndarray,
            mesh_path: Path,
            mesh_scale: np.ndarray,
        ) -> None:
            assert len(mesh_scale) == 3, f"len(mesh_scale): {len(mesh_scale)}"

            self.name = name
            self.body_id = body_id
            self.link_id = link_id
            self.mesh_path = mesh_path
            self.mesh_scale = (mesh_scale[0], mesh_scale[1], mesh_scale[2])

            link_position, link_R, _, _ = decompose(link_origin)
            quat_xyzw = mat2quat(link_R)[[1, 2, 3, 0]]
            self.link_position = link_position
            self.link_orientation = quat_xyzw

        def transform(
            self, position: np.ndarray, orientation: np.ndarray
        ) -> np.ndarray:
            return p.multiplyTransforms(
                position,
                orientation,
                self.link_position,
                self.link_orientation,
            )

        def get_keyframe(self) -> Frame:
            if self.link_id == -1:
                position, orientation = p.getBasePositionAndOrientation(self.body_id)
                position, orientation = self.transform(
                    position=position, orientation=orientation
                )
            else:
                link_state = p.getLinkState(
                    self.body_id, self.link_id, computeForwardKinematics=True
                )
                position, orientation = self.transform(
                    position=link_state[4], orientation=link_state[5]
                )
            return Frame.from_dict(
                {"position": list(position), "orientation": list(orientation)}
            )

    def __init__(self) -> None:
        self.frames: List[Dict[str, Frame]] = []
        self.link_trackers: List[PyBulletRecorder.LinkTracker] = []

    def register_object(
        self, body_id: int, urdf_path: Path, global_scaling: float = 1
    ) -> None:
        link_name_to_id = dict()
        n = p.getNumJoints(body_id)
        link_name_to_id[p.getBodyInfo(body_id)[0].decode("gb2312")] = -1
        for link_id in range(n):
            link_name_to_id[p.getJointInfo(body_id, link_id)[12].decode("gb2312")] = (
                link_id
            )

        urdf_parent_folder = urdf_path.parent.absolute()
        object_name = urdf_path.stem

        robot = URDF.load(str(urdf_path))

        for link in robot.links:
            link_id = link_name_to_id[link.name]
            if len(link.visuals) == 0:
                continue

            for i, link_visual in enumerate(link.visuals):
                mesh_scale = (
                    np.array([global_scaling, global_scaling, global_scaling])
                    if link_visual.geometry.mesh.scale is None
                    else link_visual.geometry.mesh.scale * global_scaling
                )

                # If link_id == -1 then is base link,
                # PyBullet will return
                # inertial_origin @ visual_origin,
                # so need to undo that transform
                link_origin = (
                    (
                        np.linalg.inv(link.inertial.origin)
                        if link_id == -1
                        else np.identity(4)
                    )
                    @ link_visual.origin
                    * global_scaling
                )

                mesh_path = urdf_parent_folder / link_visual.geometry.mesh.filename

                self.link_trackers.append(
                    PyBulletRecorder.LinkTracker(
                        name=object_name + f"_{body_id}_{link.name}_{i}",
                        body_id=body_id,
                        link_id=link_id,
                        link_origin=link_origin,
                        mesh_path=mesh_path,
                        mesh_scale=mesh_scale,
                    )
                )

    def add_keyframe(self) -> None:
        self.frames.append(
            {link.name: link.get_keyframe() for link in self.link_trackers}
        )

    def reset(self) -> None:
        self.frames = []

    def get_trajectories(self) -> Dict[str, Trajectory]:
        trajectories = {}
        for link in self.link_trackers:
            trajectories[link.name] = Trajectory(
                type="mesh",
                mesh_path=link.mesh_path,
                mesh_scale=link.mesh_scale,
                frames=[frame[link.name] for frame in self.frames],
            )
        return trajectories

    def save(self, path: Path) -> None:
        print("[Recorder] Saving trajectories to {}".format(path))
        pickle.dump(
            {
                link_name: trajectory.to_dict()
                for link_name, trajectory in self.get_trajectories().items()
            },
            open(path, "wb"),
        )
