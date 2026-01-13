bl_info = {
    "name": "Drone Swarm Animator",
    "author": "HA14H",
    "version": (1, 2),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > Drone Swarm",
    "description": "Create LightBender drones, animate LEDs, and export YAML",
    "category": "Animation",
}

import bpy
import math
from bpy.props import FloatProperty, StringProperty, EnumProperty, BoolProperty, IntProperty, PointerProperty
from bpy.app.handlers import persistent
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
        description="Python expression for LEDs. Vars: i (index), t (time), N (total)",
        default="[255, 255, 255] if i < t*10 else [0,0,0]"
    )

    # Export Settings (Scene Level)
    export_rate: FloatProperty(
        name="Export Rate (Hz)",
        description="Frequency of waypoints in the output file (1/delta_t)",
        default=2.0,
        min=0.1
    )


# ------------------------------------------------------------------------
#    Material Helper
# ------------------------------------------------------------------------

def get_led_material():
    """Returns a material that emits light based on Object Color"""
    mat_name = "Drone_LED_Material"
    mat = bpy.data.materials.get(mat_name)
    if not mat:
        mat = bpy.data.materials.new(name=mat_name)
        mat.use_nodes = True
        nodes = mat.node_tree.nodes
        links = mat.node_tree.links
        nodes.clear()

        # Create Nodes
        node_out = nodes.new(type='ShaderNodeOutputMaterial')
        node_emit = nodes.new(type='ShaderNodeEmission')
        node_obj_info = nodes.new(type='ShaderNodeObjectInfo')

        # Link Object Color -> Emission Color -> Surface
        links.new(node_obj_info.outputs['Color'], node_emit.inputs['Color'])
        links.new(node_emit.outputs['Emission'], node_out.inputs['Surface'])

        # Set default strength high enough to glow
        node_emit.inputs['Strength'].default_value = 5.0
    return mat


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

        # Get Material
        led_mat = get_led_material()

        # 3. Create Rod 1 (26 LEDs)
        bpy.ops.mesh.primitive_cube_add(size=1, location=(0, ROD_LEN / 2, 0))
        rod1 = context.active_object
        rod1.name = "Rod_1"
        rod1.scale = (ROD_THICKNESS, ROD_LEN, ROD_THICKNESS)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')
        rod1.parent = drone_base

        # Add visual "LEDs" to Rod 1 (Indices 0-25)
        for i in range(26):
            bpy.ops.mesh.primitive_cube_add(size=LED_SIZE)
            led = context.active_object
            led.name = f"R1_LED_{i}"

            # Store Index for animation
            led["led_index"] = i
            led.data.materials.append(led_mat)

            # 26th (index 25) is at center (0).
            dist_from_center = (25 - i) * LED_SPACING
            y_pos = dist_from_center
            led.location = (ROD_THICKNESS / 2 + LED_SIZE / 2, y_pos, 0)
            led.parent = rod1

        # 4. Create Rod 2 (24 LEDs)
        bpy.ops.mesh.primitive_cube_add(size=1, location=(0, ROD_LEN / 2, 0))
        rod2 = context.active_object
        rod2.name = "Rod_2"
        rod2.scale = (ROD_THICKNESS, ROD_LEN, ROD_THICKNESS)
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')
        rod2.parent = drone_base

        # Add visual "LEDs" to Rod 2 (Indices 26-49)
        for i in range(24):
            bpy.ops.mesh.primitive_cube_add(size=LED_SIZE)
            led = context.active_object
            led.name = f"R2_LED_{i}"

            # Store Index (Continuous: 26 + i)
            led["led_index"] = 26 + i
            led.data.materials.append(led_mat)

            # i=0 is center.
            y_pos = ((i + 1) * LED_SPACING)
            led.location = (ROD_THICKNESS / 2 + LED_SIZE / 2, y_pos, 0)
            led.parent = rod2

        # -------------------------------------------------------------
        # 5. Add Drivers for Rotation
        # -------------------------------------------------------------
        d = rod1.driver_add("rotation_euler", 0)
        var = d.driver.variables.new()
        var.name = "angle"
        var.type = 'SINGLE_PROP'
        var.targets[0].id = drone_base
        var.targets[0].data_path = '["servo_1"]'
        d.driver.expression = "-radians(angle)"

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
#    Animation Handler (Live Updates)
# ------------------------------------------------------------------------

def evaluate_leds(scene):
    """Iterates all drones and updates LED colors based on formula"""
    fps = scene.render.fps
    # Avoid div by zero
    t = (scene.frame_current - scene.frame_start) / fps if fps else 0
    if t < 0: t = 0

    # 1. Find all Drone Bases
    # We scan objects that have our specific property group data
    drones = [obj for obj in scene.objects if "servo_1" in obj and "servo_2" in obj]

    for drone in drones:
        formula = drone.drone_props.led_formula
        if not formula: continue

        # Traverse children to find LEDs
        # Hierarchy: Drone -> Rods -> LEDs
        for rod in drone.children:
            if not (rod.name.startswith("Rod_1") or rod.name.startswith("Rod_2")):
                continue

            for led in rod.children:
                # Check if it's an LED by looking for our index prop
                if "led_index" not in led:
                    continue

                i = led["led_index"]  # 0-49
                N = 50  # Total LEDs

                # Context for eval
                # We allow math module functions (sin, cos, etc)
                ctx = {"i": i, "t": t, "N": N, "math": math}

                try:
                    # Evaluate user string
                    # Expected result: [R, G, B] (0-255 or 0-1)
                    raw_color = eval(formula, {"__builtins__": None}, ctx)

                    if isinstance(raw_color, (list, tuple)) and len(raw_color) >= 3:
                        r, g, b = raw_color[0], raw_color[1], raw_color[2]

                        # Normalize 0-255 to 0-1 for Blender
                        # Heuristic: If any value > 1.0, assume 0-255 scale
                        if max(r, g, b) > 1.0:
                            r /= 255.0
                            g /= 255.0
                            b /= 255.0

                        # Clamp
                        r = max(0.0, min(1.0, r))
                        g = max(0.0, min(1.0, g))
                        b = max(0.0, min(1.0, b))

                        # Set Object Color (RGBA)
                        led.color = (r, g, b, 1.0)

                except Exception as e:
                    # Fail silently to avoid spamming console during playback
                    pass


@persistent
def update_leds_handler(scene):
    evaluate_leds(scene)


class DRONE_OT_force_update(bpy.types.Operator):
    """Force update of LED colors"""
    bl_idname = "drone.force_update_leds"
    bl_label = "Update LEDs"

    def execute(self, context):
        evaluate_leds(context.scene)
        # Trigger redraw of viewports
        for area in context.screen.areas:
            if area.type == 'VIEW_3D':
                area.tag_redraw()
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
        drones_to_export = [obj for obj in context.selected_objects if "servo_1" in obj and "servo_2" in obj]

        if not drones_to_export:
            self.report({'ERROR'}, "No Drones selected")
            return {'CANCELLED'}

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

        for drone in drones_to_export:
            output_lines.append(f"  {drone.name}:")
            waypoints = []
            servos = []

            current_frame = start_frame
            while current_frame <= end_frame:
                scene.frame_set(int(current_frame))

                loc = drone.matrix_world.translation
                x, y, z = round(loc.x, 4), round(loc.y, 4), round(loc.z, 4)
                rot_euler = drone.matrix_world.to_euler('XYZ')
                yaw = round(rot_euler.z, 4)

                waypoints.append(f"[{x}, {y}, {z}, {yaw}]")
                s1 = round(drone["servo_1"], 2)
                s2 = round(drone["servo_2"], 2)
                servos.append(f"[{s1}, {s2}]")
                current_frame += step

            output_lines.append(f"    target: {waypoints[0]}")
            output_lines.append(f"    waypoints: [{', '.join(waypoints)}]")
            output_lines.append(f"    delta_t: {delta_t}")
            output_lines.append(f"    iterations: 1")
            output_lines.append(f"    params:")
            output_lines.append(f"      linear: true")
            output_lines.append(f"      relative: false")
            output_lines.append(f"    servos: [{', '.join(servos)}]")
            output_lines.append(f"    led:")
            output_lines.append(f"      mode: \"expression\"")
            output_lines.append(f"      rate: 50")
            safe_formula = drone.drone_props.led_formula.replace('"', '\\"')
            output_lines.append(f"      formula: \"{safe_formula}\"")

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

        layout.label(text="Add Drone:")
        row = layout.row()
        row.prop(scene.drone_props, "drone_type", text="")
        row.operator("drone.add_drone", text="Create").drone_type = scene.drone_props.drone_type

        layout.separator()

        if obj and "servo_1" in obj:
            layout.label(text=f"Selected: {obj.name}")
            col = layout.column(align=True)
            col.label(text="Servo Angles (deg):")
            col.prop(obj, '["servo_1"]', text="Rod 1")
            col.prop(obj, '["servo_2"]', text="Rod 2")

            layout.separator()
            layout.label(text="LED Configuration:")
            layout.prop(obj.drone_props, "led_formula", text="")
            layout.operator("drone.force_update_leds", text="Test/Update LEDs", icon='LIGHT')
        else:
            layout.label(text="Select a drone to animate", icon='INFO')

        layout.separator()

        layout.label(text="Export Config:")
        layout.prop(scene.drone_props, "export_rate")
        layout.operator("drone.export_yaml", text="Export to YAML", icon='EXPORT')


# ------------------------------------------------------------------------
#    Registration
# ------------------------------------------------------------------------

classes = (
    DroneProperties,
    OBJECT_OT_add_drone,
    DRONE_OT_force_update,
    EXPORT_OT_drone_yaml,
    VIEW3D_PT_drone_swarm,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.drone_props = PointerProperty(type=DroneProperties)
    bpy.types.Object.drone_props = PointerProperty(type=DroneProperties)

    # Register Handler
    if update_leds_handler not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(update_leds_handler)


def unregister():
    # Unregister Handler
    if update_leds_handler in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(update_leds_handler)

    del bpy.types.Scene.drone_props
    del bpy.types.Object.drone_props

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()