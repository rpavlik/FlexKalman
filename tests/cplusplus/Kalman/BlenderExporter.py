# Copyright 2020 Collabora, Ltd.
#
# SPDX-License-Identifier: BSL-1.0 OR Apache-2.0

# This provides two exporters for blender, intended to be used on an object that's animated.
# One produces simulated measurements (the position can be from the overall object or from its first child at an offset)
# The other produces expected output, in a similar format to the data-driven test program,
# for comparison against real outputs.

import bpy
from bpy.types import Operator
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy_extras.io_utils import ExportHelper
import csv
bl_info = {
    "name": "FlexKalman Test Data Exporter",
    "blender": (2, 81, 0),
    "category": "Import-Export",
}


REAL_PREFIX = 'real_'
DT = 0.1
POSE_COLS = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', ]
MEAS_COLS = ['type', 'dt', 'x', 'y', 'z', 'w', 'off_x',
             'off_y', 'off_z'] + [REAL_PREFIX + x for x in POSE_COLS]
EXPECTED_COLS = ['time', ] + POSE_COLS


def make_vec_dict(v):
    return dict(zip(('x', 'y', 'z'), v.to_tuple()))


def make_offset_vec_dict(v):
    return dict(zip(('off_x', 'off_y', 'off_z'), v.to_tuple()))


def make_quat_measurement_dict(q):
    return {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w}


def make_quat_dict(q):
    return {'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w}


def make_real(cols):
    return {REAL_PREFIX + k: v for k, v in cols.items()}


def write_measurements(context, filepath, ball_setting):
    print("running write_measurements... use child position?", ball_setting)
    scene = context.scene
    orientation_object = context.object
    dt = 1.0/context.scene.render.fps

    if ball_setting and context.object.children:
        pos_object = context.object.children[0]
        pos_starting = {'type': 'posLever'}
        t, _, _ = pos_object.matrix_local.decompose()
        pos_starting.update(make_offset_vec_dict(t))
    else:
        pos_starting = {'type': 'pos'}
        pos_object = context.object
    with open(filepath, 'w', encoding='utf-8', newline='') as fp:
        writer = csv.DictWriter(fp, MEAS_COLS)
        writer.writeheader()
        for i in range(0, scene.frame_end):
            scene.frame_set(i)
            t, r, _ = orientation_object.matrix_world.decompose()
            real_data = make_vec_dict(t)
            real_data.update(make_quat_dict(r))
            real_data = make_real(real_data)

            t, _, _ = pos_object.matrix_world.decompose()
            row = {'dt': dt}
            row.update(real_data)
            row.update(pos_starting)
            row.update(make_vec_dict(t))
            writer.writerow(row)

            _, r, _ = orientation_object.matrix_world.decompose()
            row = {'type': 'quat', 'dt': 0}
            row.update(real_data)
            row.update(make_quat_measurement_dict(r))
            writer.writerow(row)


def write_expected(context, filepath):
    print("running write_expected...")
    scene = context.scene
    obj = context.object
    dt = 1.0/context.scene.render.fps

    with open(filepath, 'w', encoding='utf-8', newline='') as fp:
        writer = csv.DictWriter(fp, EXPECTED_COLS)
        writer.writeheader()
        timestamp = 0
        for i in range(0, scene.frame_end):
            timestamp += dt
            scene.frame_set(i)
            t, r, _ = obj.matrix_world.decompose()
            row = {'time': (i + 1) * dt}
            row.update(make_vec_dict(t))
            row.update(make_quat_dict(r))
            writer.writerow(row)


# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.


class ExportMeasurements(Operator, ExportHelper):
    """Export simulated tracker measurements from an object"""
    bl_idname = "export_poses.measurements"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Export simulated pose measurements"

    # ExportHelper mixin class uses this
    filename_ext = ".csv"

    filter_glob: StringProperty(
        default="*.csv",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    ball_setting: BoolProperty(
        name="Use position (offset) of child",
        description="Export the position (as posLever) of the first child instead of the selected object",
        default=False,
    )

    def execute(self, context):
        write_measurements(context, self.filepath, self.ball_setting)
        return {'FINISHED'}


class ExportPoses(Operator, ExportHelper):
    """Export expected (tracked) poses from an object"""
    bl_idname = "export_poses.expected"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Export expected poses"

    # ExportHelper mixin class uses this
    filename_ext = ".csv"

    filter_glob: StringProperty(
        default="*.csv",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    def execute(self, context):
        write_expected(context, self.filepath)
        return {'FINISHED'}

# Only needed if you want to add into a dynamic menu


def menu_func_export(self, context):
    self.layout.operator(ExportMeasurements.bl_idname,
                         text="Simulated tracking measurement")
    self.layout.operator(ExportPoses.bl_idname,
                         text="Expected tracking poses")


def register():
    bpy.utils.register_class(ExportMeasurements)
    bpy.utils.register_class(ExportPoses)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(ExportMeasurements)
    bpy.utils.unregister_class(ExportPoses)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()

    # test call
#    bpy.ops.export_test.some_data('INVOKE_DEFAULT')
