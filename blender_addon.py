bl_info = {
    "name": "LightBender Swarm Animator",
    "author": "HA",
    "version": (1, 8),
    "blender": (3, 0, 0),
    "location": "View3D > Sidebar > LightBender",
    "description": "Create light-element drones, animate LEDs with pointers, and export YAML",
    "category": "Animation",
}

import bpy
import math
from bpy.props import FloatProperty, StringProperty, EnumProperty, BoolProperty, IntProperty, PointerProperty, \
    CollectionProperty
from bpy.app.handlers import persistent
from mathutils import Vector, Matrix


# ------------------------------------------------------------------------
#    Properties & Data Classes
# ------------------------------------------------------------------------

class LEDPointer(bpy.types.PropertyGroup):
    """Defines a split point on the LED strip and the color following it"""
    # Position on the strip (0 to 50)
    value: FloatProperty(
        name="Position",
        description="Index location of the pointer",
        default=10.0,
        soft_min=0.0,
        soft_max=50.0
    )
    # Color expression for the segment starting at this pointer
    color_expression: StringProperty(
        name="Color",
        description="Color expression for LEDs after this pointer",
        default="[0, 255, 0]"
    )


class DroneProperties(bpy.types.PropertyGroup):
    drone_type: EnumProperty(
        name="Type",
        description="Light Element Type",
        items=[
            ('TYPE_H', "Type H (0-180, 180-360)", "Segment 1: 0-180, Segment 2: 180-360"),
            ('TYPE_V', "Type V (90-270, 270-450)", "Segment 1: 90-270, Segment 2: 270-450"),
        ],
        default='TYPE_H'
    )

    # LED Control Modes
    led_mode: EnumProperty(
        name="LED Mode",
        items=[
            ('EXPRESSION', "Single Expression", "Use one python formula for all LEDs"),
            ('POINTERS', "Pointers", "Use pointers to define colored segments"),
        ],
        default='EXPRESSION'
    )

    # Mode A: Single Expression
    led_formula: StringProperty(
        name="Formula",
        description="Python expression for LEDs. Vars: i (index), t (time), N (total)",
        default="[255, 255, 255] if i < t*10 else [0,0,0]"
    )

    # Mode B: Pointers
    led_base_color: StringProperty(
        name="Base Color",
        description="Color expression for LEDs before the first pointer",
        default="[0, 0, 0]"
    )

    led_pointers: CollectionProperty(type=LEDPointer)
    active_pointer_index: IntProperty(name="Active Pointer", default=0)

    # Export Settings (Scene Level)
    export_rate: FloatProperty(
        name="Export Rate (Hz)",
        description="Frequency of waypoints in the output file (1/delta_t)",
        default=2.0,
        min=0.1
    )

    export_at_keyframes: BoolProperty(
        name="Export Keyframes Only",
        description="Export only at keyframe locations (adds dt to waypoints)",
        default=False
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
            ('TYPE_H', "Type H", "Segment 1: 0-180, Segment 2: 180-360"),
            ('TYPE_V', "Type V", "Segments 1: 90-270, Segment 2: 270-450"),
        ],
        default='TYPE_H'
    )

    def execute(self, context):
        # 1. Create Main Drone Body (Empty - No Geometry)
        bpy.ops.object.empty_add(type='ARROWS', location=(0, 0, 0))
        drone_base = context.active_object
        drone_base.name = "Drone_Base"

        # Add Custom Properties to Base for Animation
        drone_base["servo_1"] = 0.0
        drone_base["servo_2"] = 180.0 if self.drone_type == 'TYPE_H' else 270.0

        # UI Property definitions for limits
        mgr = drone_base.id_properties_ui("servo_1")
        if self.drone_type == 'TYPE_H':
            mgr.update(min=0.0, max=180.0, soft_min=0.0, soft_max=180.0)
        else:
            mgr.update(min=90.0, max=270.0, soft_min=90.0, soft_max=270.0)

        mgr = drone_base.id_properties_ui("servo_2")
        if self.drone_type == 'TYPE_H':
            mgr.update(min=180.0, max=360.0, soft_min=180.0, soft_max=360.0)
        else:  # Type V
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
#    Pointer List UI Operators
# ------------------------------------------------------------------------

class DRONE_OT_add_pointer(bpy.types.Operator):
    """Add a new LED Pointer"""
    bl_idname = "drone.add_pointer"
    bl_label = "Add Pointer"

    def execute(self, context):
        obj = context.active_object
        if not obj: return {'CANCELLED'}

        # Add new item
        item = obj.drone_props.led_pointers.add()
        # Default value slightly offset to avoid overlap
        item.value = 10.0 + (len(obj.drone_props.led_pointers) * 5)
        item.color_expression = "[255, 0, 0]"

        obj.drone_props.active_pointer_index = len(obj.drone_props.led_pointers) - 1
        return {'FINISHED'}


class DRONE_OT_remove_pointer(bpy.types.Operator):
    """Remove selected LED Pointer"""
    bl_idname = "drone.remove_pointer"
    bl_label = "Remove Pointer"

    def execute(self, context):
        obj = context.active_object
        if not obj: return {'CANCELLED'}

        idx = obj.drone_props.active_pointer_index
        obj.drone_props.led_pointers.remove(idx)

        if idx >= len(obj.drone_props.led_pointers):
            obj.drone_props.active_pointer_index = max(0, len(obj.drone_props.led_pointers) - 1)

        return {'FINISHED'}


class UL_DronePointerList(bpy.types.UIList):
    """List representation of Pointers"""

    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        # Draw layout: Position Value | Color Expression
        split = layout.split(factor=0.3)
        split.prop(item, "value", text="", emboss=False)  # Editable number
        split.prop(item, "color_expression", text="", emboss=False)


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
    drones = [obj for obj in scene.objects if "servo_1" in obj and "servo_2" in obj]

    for drone in drones:
        props = drone.drone_props
        mode = props.led_mode

        # Prepare Pointer Data if in Pointer Mode
        sorted_pointers = []
        base_expr = ""

        if mode == 'POINTERS':
            base_expr = props.led_base_color
            # Collect pointers and sort by current value
            # We must evaluate value here because it might be animated
            # Note: Animation on collection items requires evaluating the property
            # For simplicity in this handler, we read the current property value which Blender updates

            raw_pointers = []
            for ptr in props.led_pointers:
                raw_pointers.append({
                    'val': ptr.value,
                    'expr': ptr.color_expression
                })

            # Sort by position value
            sorted_pointers = sorted(raw_pointers, key=lambda x: x['val'])
        else:
            # Expression Mode
            base_expr = props.led_formula

        # Traverse children to find LEDs
        # Hierarchy: Drone -> Rods -> LEDs
        for rod in drone.children:
            if not (rod.name.startswith("Rod_1") or rod.name.startswith("Rod_2")):
                continue

            for led in rod.children:
                if "led_index" not in led:
                    continue

                i = led["led_index"]  # 0-49
                N = 50  # Total LEDs

                # Determine formula for this specific LED based on Mode
                active_formula = ""

                if mode == 'EXPRESSION':
                    active_formula = base_expr
                elif mode == 'POINTERS':
                    # Find interval
                    # Default to base
                    active_formula = base_expr

                    # Iterate sorted pointers to find which segment we are in
                    # Logic: if i >= pointer.value, we adopt that pointer's color
                    # Since sorted, the last one we satisfy is the correct one
                    for ptr in sorted_pointers:
                        if i >= ptr['val']:
                            active_formula = ptr['expr']
                        else:
                            # Since sorted, if i < current, it is < all subsequent
                            break

                # Evaluate
                ctx = {"i": i, "t": t, "N": N, "math": math}

                try:
                    raw_color = eval(active_formula, {}, ctx)

                    if isinstance(raw_color, (list, tuple)) and len(raw_color) >= 3:
                        r, g, b = raw_color[0], raw_color[1], raw_color[2]

                        # Normalize
                        if max(r, g, b) > 1.0:
                            r /= 255.0
                            g /= 255.0
                            b /= 255.0

                        # Clamp
                        r = max(0.0, min(1.0, r))
                        g = max(0.0, min(1.0, g))
                        b = max(0.0, min(1.0, b))

                        led.color = (r, g, b, 1.0)

                except Exception as e:
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

        # Export Settings
        use_keyframes = scene.drone_props.export_at_keyframes

        if use_keyframes:
            # Mode A: Keyframes Only
            delta_t = 0.0
        else:
            # Mode B: Fixed Rate
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
            pointer_data = []

            # --- Frame Collection ---
            frames_to_export = []
            if use_keyframes:
                keys = set()
                if drone.animation_data and drone.animation_data.action:
                    for fcurve in drone.animation_data.action.fcurves:
                        for kp in fcurve.keyframe_points:
                            if start_frame <= kp.co.x <= end_frame:
                                keys.add(kp.co.x)
                keys.add(start_frame)
                keys.add(end_frame)
                frames_to_export = sorted(list(keys))
            else:
                curr = float(start_frame)
                while curr <= end_frame:
                    frames_to_export.append(curr)
                    curr += step

            # --- Data Collection ---
            prev_time_sec = 0.0

            for i, frame in enumerate(frames_to_export):
                scene.frame_set(int(frame))
                current_time_sec = (frame - start_frame) / fps

                if i == 0:
                    dt_val = 0.0
                else:
                    dt_val = current_time_sec - prev_time_sec
                prev_time_sec = current_time_sec

                loc = drone.matrix_world.translation
                x, y, z = round(loc.x, 4), round(loc.y, 4), round(loc.z, 4)
                rot_euler = drone.matrix_world.to_euler('XYZ')
                yaw = round(rot_euler.z, 4)
                s1 = round(drone["servo_1"], 2)
                s2 = round(drone["servo_2"], 2)

                # Capture Pointers
                ptrs = [round(p.value, 2) for p in drone.drone_props.led_pointers]
                pointer_data.append(f"[{', '.join(map(str, ptrs))}]")

                if use_keyframes:
                    waypoints.append(f"[{x}, {y}, {z}, {yaw}, {round(dt_val, 4)}]")
                    servos.append(f"[{s1}, {s2}]")
                else:
                    waypoints.append(f"[{x}, {y}, {z}, {yaw}]")
                    servos.append(f"[{s1}, {s2}]")

            # --- Write Block ---
            output_lines.append(f"    target: {waypoints[0]}")
            output_lines.append(f"    waypoints: [{', '.join(waypoints)}]")
            output_lines.append(f"    delta_t: {delta_t}")
            output_lines.append(f"    iterations: 1")
            output_lines.append(f"    params:")
            output_lines.append(f"      linear: true")
            output_lines.append(f"      relative: false")
            output_lines.append(f"    servos: [{', '.join(servos)}]")
            output_lines.append(f"    pointers: [{', '.join(pointer_data)}]")

            # --- LED Formula Generation ---
            props = drone.drone_props
            final_formula = ""

            if props.led_mode == 'EXPRESSION':
                final_formula = props.led_formula
            elif props.led_mode == 'POINTERS':
                # Compile pointers into a single nested Python ternary expression using variables p0, p1...

                raw_ptrs = []
                for idx, ptr in enumerate(props.led_pointers):
                    # Store index-based variable name 'p{idx}' and value/color
                    raw_ptrs.append({'id': f"p{idx}", 'v': ptr.value, 'c': ptr.color_expression})

                # Sort by value to establish the logical nesting hierarchy (A < B < C)
                # We use the current frame's values to determine the structure of the ternary nest.
                raw_ptrs.sort(key=lambda x: x['v'])

                if not raw_ptrs:
                    final_formula = props.led_base_color
                else:
                    # Start with the last "else" which is the color of the last pointer
                    current_str = f"({raw_ptrs[-1]['c']})"

                    # Iterate backwards
                    for j in range(len(raw_ptrs) - 2, -1, -1):
                        p_curr = raw_ptrs[j]
                        p_next = raw_ptrs[j + 1]
                        # Use variable name {p_next['id']} instead of literal value
                        current_str = f"({p_curr['c']}) if i < {p_next['id']} else {current_str}"

                    p0 = raw_ptrs[0]
                    final_formula = f"({props.led_base_color}) if i < {p0['id']} else {current_str}"

            output_lines.append(f"    led:")
            output_lines.append(f"      mode: \"expression\"")
            output_lines.append(f"      rate: 50")
            safe_formula = final_formula.replace('"', '\\"')
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
    bl_category = "LightBender"
    bl_label = "LightBender Controls"

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
            props = obj.drone_props

            layout.label(text=f"Selected: {obj.name}")
            col = layout.column(align=True)
            col.label(text="Servo Angles (deg):")
            col.prop(obj, '["servo_1"]', text="Angle 1")
            col.prop(obj, '["servo_2"]', text="Angle 2")

            layout.separator()
            layout.label(text="LED Configuration:")
            layout.prop(props, "led_mode", text="Mode")

            if props.led_mode == 'EXPRESSION':
                layout.prop(props, "led_formula", text="")
            else:
                # Pointer Mode UI
                box = layout.box()
                box.label(text="Base Color (Start):")
                box.prop(props, "led_base_color", text="")

                box.label(text="Pointers (Split Points):")

                row = box.row()
                row.template_list("UL_DronePointerList", "", props, "led_pointers", props, "active_pointer_index")

                col = row.column(align=True)
                col.operator("drone.add_pointer", icon='ADD', text="")
                col.operator("drone.remove_pointer", icon='REMOVE', text="")

                # Hint for usage
                box.label(text="Pos | Right-Side Color", icon='INFO')

            layout.separator()
            layout.operator("drone.force_update_leds", text="Test/Update LEDs", icon='LIGHT')
        else:
            layout.label(text="Select a drone to animate", icon='INFO')

        layout.separator()

        layout.label(text="Export Config:")
        layout.prop(scene.drone_props, "export_at_keyframes")

        if not scene.drone_props.export_at_keyframes:
            layout.prop(scene.drone_props, "export_rate")

        layout.operator("drone.export_yaml", text="Export to YAML", icon='EXPORT')


# ------------------------------------------------------------------------
#    Registration
# ------------------------------------------------------------------------

classes = (
    LEDPointer,
    DroneProperties,
    OBJECT_OT_add_drone,
    DRONE_OT_add_pointer,
    DRONE_OT_remove_pointer,
    UL_DronePointerList,
    DRONE_OT_force_update,
    EXPORT_OT_drone_yaml,
    VIEW3D_PT_drone_swarm,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Scene.drone_props = PointerProperty(type=DroneProperties)
    bpy.types.Object.drone_props = PointerProperty(type=DroneProperties)

    if update_leds_handler not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(update_leds_handler)


def unregister():
    if update_leds_handler in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(update_leds_handler)

    del bpy.types.Scene.drone_props
    del bpy.types.Object.drone_props

    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()