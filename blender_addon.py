bl_info = {
    "name": "Drone Swarm Animator",
    "author": "Gemini",
    "version": (1, 1),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Drone Swarm",
    "description": "Create light-element drones and export flight paths to YAML",
    "category": "Animation",
}

import bpy
import math
from bpy.props import FloatProperty, StringProperty, EnumProperty, BoolProperty, IntProperty, PointerProperty
from mathutils import Vector, Matrix


# ------------------------------------------------------------------------
#    Properties & Data Classes
# ------------------------------------------------------------------------

class DroneProperties(bpy.types.PropertyGroup):
    drone_type: EnumProperty(
        name="Type",
        description="Light Element Type",
        items=[
            ('TYPE_A', "Type A (0-180, 180-360)", "Rod 1: 0-180, Rod 2: 180-360"),
            ('TYPE_B', "Type B (90-270, 270-450)", "Rod 1: 90-270, Rod 2: 270-450"),
        ],
        default='TYPE_A'
    )

    led_formula: StringProperty(
        name="LED Formula",
        description="Python expression for LEDs",
        default="[255, 255, 255] if i < t else [0,0,0]"
    )

    # Export Settings (Scene Level)
    export_rate: FloatProperty(
        name="Export Rate (Hz)",
        description="Frequency of waypoints in the output file (1/delta_t)",
        default=2.0,
        min=0.1
    )


# ------------------------------------------------------------------------
#    Geometry & Setup Operator
# ------------------------------------------------------------------------

class OBJECT_OT_add_drone(bpy.types.Operator):
    """Create a new Drone with Light Elements"""
    bl_idname = "drone.add_drone"
    bl_label = "Add Drone"
    bl_options = {'REGISTER', 'UNDO'}

    drone_type: EnumProperty(
        name="Type",
        items=[
            ('TYPE_A', "Type A", "Rod 1: 0-180, Rod 2: 180-360"),
            ('TYPE_B', "Type B", "Type B: 90-270, Rod 2: 270-450"),
        ],
        default='TYPE_A'
    )

    def execute(self, context):
        # 1. Create Main Drone Body (Empty - No Geometry)
        bpy.ops.object.empty_add(type='ARROWS', location=(0, 0, 0))
        drone_base = context.active_object
        drone_base.name = "Drone_Base"
        # Rotate so Tip points +X (Forward) - Empties point Y forward by default in some views,
        # but pure Empty has local axes. We leave it at 0,0,0 identity for simplicity,
        # or rotate 90 Z if we want Local X to be 'forward' relative to world Y.
        # User spec: "rotation axis is aligned with the forward direction of the drone".
        # We assume Local X is the rotation axis.

        # Add Custom Properties to Base for Animation
        drone_base["servo_1"] = 0.0
        drone_base["servo_2"] = 180.0 if self.drone_type == 'TYPE_A' else 270.0

        # UI Property definitions for limits
        mgr = drone_base.id_properties_ui("servo_1")
        if self.drone_type == 'TYPE_A':
            mgr.update(min=0.0, max=180.0, soft_min=0.0, soft_max=180.0)
        else:
            mgr.update(min=90.0, max=270.0, soft_min=90.0, soft_max=270.0)

        mgr = drone_base.id_properties_ui("servo_2")
        if self.drone_type == 'TYPE_A':
            mgr.update(min=180.0, max=360.0, soft_min=180.0, soft_max=360.0)
        else:  # Type B
            mgr.update(min=270.0, max=450.0, soft_min=270.0, soft_max=450.0)

        # Attach our custom property group
        drone_base.drone_props.drone_type = self.drone_type

        # Constants
        ROD_LEN = 0.15  # 15 cm
        LED_SIZE = 0.002  # 2 mm
        LED_SPACING = 0.00624  # 6.24 mm
        ROD_THICKNESS = 0.003

        # 3. Create Rod 1 (26 LEDs)
        # "rod_1 rotates from 0 to 180" -> 0 is 3 o'clock.
        # In Blender (X forward), 3 o'clock is -Y.
        # We create geometry such that it extends along -Y, with Origin at (0,0,0).

        # Center of a 15cm rod starting at 0 and going to -0.15 is -0.075
        bpy.ops.mesh.primitive_cube_add(size=1, location=(0, ROD_LEN / 2, 0))
        rod1 = context.active_object
        rod1.name = "Rod_1"
        rod1.scale = (ROD_THICKNESS, ROD_LEN, ROD_THICKNESS)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')
        rod1.parent = drone_base

        # Add visual "LEDs" to Rod 1
        # "rod_1 has 26 leds... the 26th led is aligned with the rotation center"
        # Spaced 6mm apart.
        # i=0 (Tip) to i=25 (Center).
        for i in range(26):
            bpy.ops.mesh.primitive_cube_add(size=LED_SIZE)
            led = context.active_object
            led.name = f"R1_LED_{i}"

            # 26th (index 25) is at center (0).
            # Index i distance from center = (25 - i) * 0.006
            # Direction is along -Y (since rod is at 3 o'clock)
            dist_from_center = (25 - i) * LED_SPACING
            y_pos = dist_from_center

            # "placing the leds facing toward the x axis"
            # Offset slightly in X so they sit on the face of the rod
            led.location = (ROD_THICKNESS / 2 + LED_SIZE / 2, y_pos, 0)
            led.parent = rod1

        # 4. Create Rod 2 (24 LEDs)
        # "rod_2 has 24 leds extending from center to its end point"
        # We model it in the rest position (-Y/3 o'clock) same as Rod 1,
        # the servo driver handles the initial offset (180 deg).

        # Geometry for Rod 2 (Same length 15cm for visual consistency, or just enough for LEDs?)
        # Let's make it 15cm as well.
        bpy.ops.mesh.primitive_cube_add(size=1, location=(0, ROD_LEN / 2, 0))
        rod2 = context.active_object
        rod2.name = "Rod_2"
        rod2.scale = (ROD_THICKNESS, ROD_LEN, ROD_THICKNESS)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')
        rod2.parent = drone_base

        # Visual separation slightly in X is handled by LED offset, but rods shouldn't Z-fight.
        # Let's offset Rod 2 slightly in X (behind Rod 1?) or just let them overlap perfectly
        # since they rotate to different angles usually.
        # Let's keep them zero-centered on Z.
        # Offset Rod 2 slightly in X to allow LEDs to pass?
        # rod2.location.x -= 0.003  # 3mm behind

        # Add visual "LEDs" to Rod 2
        for i in range(24):
            bpy.ops.mesh.primitive_cube_add(size=LED_SIZE)
            led = context.active_object
            led.name = f"R2_LED_{i}"

            # "extending from center to its end point"
            # i=0 is center.
            y_pos = ((i + 1) * LED_SPACING)

            led.location = (ROD_THICKNESS / 2 + LED_SIZE / 2, y_pos, 0)
            led.parent = rod2

        # -------------------------------------------------------------
        # 5. Add Drivers for Rotation
        # -------------------------------------------------------------

        # Driver for Rod 1 (X Axis rotation)
        d = rod1.driver_add("rotation_euler", 0)
        var = d.driver.variables.new()
        var.name = "angle"
        var.type = 'SINGLE_PROP'
        var.targets[0].id = drone_base
        var.targets[0].data_path = '["servo_1"]'
        # Clockwise rotation relative to X axis
        d.driver.expression = "-radians(angle)"

        # Driver for Rod 2
        d = rod2.driver_add("rotation_euler", 0)
        var = d.driver.variables.new()
        var.name = "angle"
        var.type = 'SINGLE_PROP'
        var.targets[0].id = drone_base
        var.targets[0].data_path = '["servo_2"]'
        d.driver.expression = "-radians(angle)"

        # Select the base
        bpy.ops.object.select_all(action='DESELECT')
        drone_base.select_set(True)
        context.view_layer.objects.active = drone_base

        return {'FINISHED'}


# ------------------------------------------------------------------------
#    Export Operator
# ------------------------------------------------------------------------

class EXPORT_OT_drone_yaml(bpy.types.Operator):
    """Export Drone Animation to YAML"""
    bl_idname = "drone.export_yaml"
    bl_label = "Export YAML"

    filepath: StringProperty(subtype="FILE_PATH")

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        scene = context.scene

        # 1. Identify Drones
        drones_to_export = []
        for obj in context.selected_objects:
            if "servo_1" in obj and "servo_2" in obj:
                drones_to_export.append(obj)

        if not drones_to_export:
            self.report({'ERROR'}, "No Drones selected (Objects with servo_1/2 properties)")
            return {'CANCELLED'}

        # 2. Setup Timing
        fps = scene.render.fps
        export_hz = scene.drone_props.export_rate
        if export_hz <= 0: export_hz = 1.0

        step = fps / export_hz
        delta_t = 1.0 / export_hz

        start_frame = scene.frame_start
        end_frame = scene.frame_end

        output_lines = []
        output_lines.append(f"name: {scene.name.replace(' ', '_')}")
        output_lines.append("drones:")

        # 3. Iterate Drones
        for drone in drones_to_export:
            output_lines.append(f"  {drone.name}:")

            # --- Capture Data ---
            waypoints = []
            servos = []

            current_frame = start_frame
            while current_frame <= end_frame:
                scene.frame_set(int(current_frame))  # Set frame to evaluate drivers/anim

                # Position (Round to 4 decimals for clean file)
                loc = drone.matrix_world.translation
                x, y, z = round(loc.x, 4), round(loc.y, 4), round(loc.z, 4)

                # Rotation (Yaw/Z-axis)
                # Ensure we get the Z rotation regardless of rotation mode
                rot_euler = drone.matrix_world.to_euler('XYZ')
                yaw = round(rot_euler.z, 4)

                waypoints.append(f"[{x}, {y}, {z}, {yaw}]")

                # Servos
                s1 = round(drone["servo_1"], 2)
                s2 = round(drone["servo_2"], 2)
                servos.append(f"[{s1}, {s2}]")

                current_frame += step

            # --- Write Data Blocks ---

            # Target (First frame state)
            output_lines.append(f"    target: {waypoints[0]}")

            # Waypoints
            output_lines.append(f"    waypoints: [{', '.join(waypoints)}]")

            # Params
            output_lines.append(f"    delta_t: {delta_t}")
            output_lines.append(f"    iterations: 1")
            output_lines.append(f"    params:")
            output_lines.append(f"      linear: true")
            output_lines.append(f"      relative: false")

            # Servos
            output_lines.append(f"    servos: [{', '.join(servos)}]")

            # LED Info
            output_lines.append(f"    led:")
            output_lines.append(f"      mode: \"expression\"")
            output_lines.append(f"      rate: 50")
            # Escape quotes in formula just in case
            safe_formula = drone.drone_props.led_formula.replace('"', '\\"')
            output_lines.append(f"      formula: \"{safe_formula}\"")

        # 4. Write File
        try:
            with open(self.filepath, 'w', encoding='utf-8') as f:
                f.write('\n'.join(output_lines))
            self.report({'INFO'}, f"Exported to {self.filepath}")
        except Exception as e:
            self.report({'ERROR'}, f"File Write Error: {str(e)}")
            return {'CANCELLED'}

        return {'FINISHED'}


# ------------------------------------------------------------------------
#    UI Panel
# ------------------------------------------------------------------------

class VIEW3D_PT_drone_swarm(bpy.types.Panel):
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Drone Swarm"
    bl_label = "Swarm Controls"

    def draw(self, context):
        layout = self.layout
        scene = context.scene
        obj = context.active_object

        # Add Section
        layout.label(text="Add Drone:")
        row = layout.row()
        row.prop(scene.drone_props, "drone_type", text="")
        row.operator("drone.add_drone", text="Create").drone_type = scene.drone_props.drone_type

        layout.separator()

        # Animation Controls (Context Sensitive)
        if obj and "servo_1" in obj:
            layout.label(text=f"Selected: {obj.name}")
            col = layout.column(align=True)
            col.label(text="Servo Angles (deg):")
            col.prop(obj, '["servo_1"]', text="Rod 1")
            col.prop(obj, '["servo_2"]', text="Rod 2")

            layout.separator()
            layout.label(text="LED Configuration:")
            layout.prop(obj.drone_props, "led_formula", text="")
        else:
            layout.label(text="Select a drone to animate", icon='INFO')

        layout.separator()

        # Export Section
        layout.label(text="Export Config:")
        layout.prop(scene.drone_props, "export_rate")
        layout.operator("drone.export_yaml", text="Export to YAML", icon='EXPORT')


# ------------------------------------------------------------------------
#    Registration
# ------------------------------------------------------------------------

classes = (
    DroneProperties,
    OBJECT_OT_add_drone,
    EXPORT_OT_drone_yaml,
    VIEW3D_PT_drone_swarm,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.drone_props = PointerProperty(type=DroneProperties)
    bpy.types.Object.drone_props = PointerProperty(type=DroneProperties)


def unregister():
    del bpy.types.Scene.drone_props
    del bpy.types.Object.drone_props

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
