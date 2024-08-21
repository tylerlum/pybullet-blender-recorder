import pickle
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import bpy
from bpy.props import CollectionProperty, StringProperty
from bpy.types import Operator, OperatorFileListElement, Panel
from bpy_extras.io_utils import ImportHelper


@dataclass
class Frame:
    position: Tuple[float, float, float]  # xyz
    orientation: Tuple[float, float, float, float]  # xyzw

    def __post_init__(self):
        assert len(self.position) == 3, f"len(self.position): {len(self.position)}"
        assert (
            len(self.orientation) == 4
        ), f"len(self.orientation): {len(self.orientation)}"

    @classmethod
    def from_dict(cls, data: Dict):
        return cls(position=data["position"], orientation=data["orientation"])

    def to_dict(self):
        return {"position": self.position, "orientation": self.orientation}


@dataclass
class Trajectory:
    type: str
    mesh_path: Path
    mesh_scale: Tuple[float, float, float]
    frames: List[Frame]

    def __post_init__(self):
        assert self.type in ["mesh"], f"self.type: {self.type}"
        assert self.mesh_path.exists(), f"self.mesh_path: {self.mesh_path}"
        assert (
            len(self.mesh_scale) == 3
        ), f"len(self.mesh_scale): {len(self.mesh_scale)}"
        assert len(self.frames) > 0, f"len(self.frames): {len(self.frames)}"

    @classmethod
    def from_dict(cls, data: Dict):
        return cls(
            type=data["type"],
            mesh_path=Path(data["mesh_path"]),
            mesh_scale=data["mesh_scale"],
            frames=[Frame.from_dict(frame) for frame in data["frames"]],
        )

    def to_dict(self):
        return {
            "type": self.type,
            "mesh_path": str(self.mesh_path),
            "mesh_scale": self.mesh_scale,
            "frames": [frame.to_dict() for frame in self.frames],
        }


bl_info = {
    "name": "PyBulletSimImporter",
    "author": "Huy Ha <hqh2101@columbia.edu>",
    "version": (0, 0, 1),
    "blender": (2, 92, 0),
    "location": "3D View > Toolbox > Animation tab > PyBullet Simulation Importer",
    "description": "Imports PyBullet Simulation Results",
    "warning": "",
    "category": "Animation",
}


class ANIM_OT_import_pybullet_sim(Operator, ImportHelper):
    bl_label = "Import simulation"
    bl_idname = "pybulletsim.import"
    bl_description = "Imports a PyBullet Simulation"
    bl_options = {"REGISTER", "UNDO"}
    files: CollectionProperty(
        name="Simulation files",
        type=OperatorFileListElement,
    )
    directory: StringProperty(subtype="DIR_PATH")
    filename_ext = ".pkl"
    filter_glob: StringProperty(default="*.pkl", options={"HIDDEN"})
    skip_frames: bpy.props.IntProperty(name="Skip Frames", default=10, min=1, max=100)
    max_frames: bpy.props.IntProperty(name="Max Frames", default=-1, min=-1, max=100000)

    def execute(self, context):
        for file in self.files:
            filepath = Path(self.directory) / file.name
            print(f"Processing {filepath}")

            with open(filepath, "rb") as pickle_file:
                raw_data = pickle.load(pickle_file)
                trajectories = {
                    link_name: Trajectory.from_dict(data_dict)
                    for link_name, data_dict in raw_data.items()
                }

                collection = bpy.data.collections.new(filepath.stem)
                bpy.context.scene.collection.children.link(collection)
                context.view_layer.active_layer_collection = (
                    context.view_layer.layer_collection.children[-1]
                )

                for link_name, trajectory in trajectories.items():
                    # Load mesh of each link
                    extension = trajectory.mesh_path.suffix

                    # Handle different mesh formats
                    if "obj" in extension:
                        bpy.ops.wm.obj_import(
                            filepath=trajectory.mesh_path,
                            forward_axis="Y",
                            up_axis="Z",
                        )
                    elif "dae" in extension:
                        bpy.ops.wm.collada_import(filepath=trajectory.mesh_path)
                    elif "stl" in extension:
                        bpy.ops.wm.stl_import(filepath=trajectory.mesh_path)
                    else:
                        print("Unsupported File Format:{}".format(extension))
                        pass

                    final_objs = []
                    for import_obj in context.selected_objects:
                        bpy.ops.object.select_all(action="DESELECT")
                        import_obj.select_set(True)
                        if (
                            "Camera" in import_obj.name
                            or "Light" in import_obj.name
                            or "Lamp" in import_obj.name
                        ):
                            # Delete lights and camera
                            bpy.ops.object.delete(use_global=True)
                        else:
                            # Set scale
                            scale = trajectory.mesh_scale
                            if scale is not None:
                                import_obj.scale.x = scale[0]
                                import_obj.scale.y = scale[1]
                                import_obj.scale.z = scale[2]
                            final_objs.append(import_obj)

                    bpy.ops.object.select_all(action="DESELECT")
                    for obj in final_objs:
                        if obj.type == "MESH":
                            obj.select_set(True)

                    if len(context.selected_objects):
                        context.view_layer.objects.active = context.selected_objects[0]
                        # join them
                        bpy.ops.object.join()

                    blender_obj = context.view_layer.objects.active
                    blender_obj.name = link_name

                    # Keyframe motion of imported object
                    for frame_i, frame in enumerate(trajectory.frames):
                        if frame_i % self.skip_frames != 0:
                            continue

                        if self.max_frames > 1 and frame_i > self.max_frames:
                            print("Exceed max frame count")
                            break

                        percentage_done = frame_i / len(trajectory.frames)
                        print(
                            f"\r[{percentage_done*100:.01f}% | {link_name}]",
                            "#" * int(60 * percentage_done),
                            end="",
                        )
                        context.scene.frame_set(frame_i // self.skip_frames)

                        # Apply position and rotation
                        blender_obj.location.x = frame.position[0]
                        blender_obj.location.x = frame.position[1]
                        blender_obj.location.x = frame.position[2]
                        blender_obj.rotation_mode = "QUATERNION"
                        blender_obj.rotation_quaternion.x = frame.orientation[0]
                        blender_obj.rotation_quaternion.y = frame.orientation[1]
                        blender_obj.rotation_quaternion.z = frame.orientation[2]
                        blender_obj.rotation_quaternion.w = frame.orientation[3]
                        bpy.ops.anim.keyframe_insert_menu(type="Rotation")
                        bpy.ops.anim.keyframe_insert_menu(type="Location")
        return {"FINISHED"}


class VIEW3D_PT_pybullet_recorder(Panel):
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animation"
    bl_label = "PyBulletSimImporter"

    def draw(self, context):
        layout = self.layout
        row = layout.row()
        row.operator("pybulletsim.import")


def register():
    bpy.utils.register_class(VIEW3D_PT_pybullet_recorder)
    bpy.utils.register_class(ANIM_OT_import_pybullet_sim)


def unregister():
    bpy.utils.unregister_class(VIEW3D_PT_pybullet_recorder)
    bpy.utils.unregister_class(ANIM_OT_import_pybullet_sim)


if __name__ == "__main__":
    register()
