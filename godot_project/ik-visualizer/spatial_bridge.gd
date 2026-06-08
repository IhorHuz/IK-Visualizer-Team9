extends Node3D

enum RendererMode { STICK, ROBOT }

const FBX_BASE = preload("res://visuals/Base_001.fbx")
const FBX_BASE_ROTATION = preload("res://visuals/Base_rotation_001.fbx")
const FBX_ARM_1 = preload("res://visuals/arm_1_001.fbx")
const FBX_ARM_2 = preload("res://visuals/arm_2_001.fbx")
const FBX_ENDPOINT = preload("res://visuals/endpoint_001.fbx")
const FBX_H_JOINT_BASE = preload("res://visuals/horizontal_joint_base_002.fbx")
const FBX_H_JOINT_RECEPTICLE = preload("res://visuals/horizontal_joint_recepticle_001.fbx")

const MODEL_SCALE := 1.0
const ACTIVE_JOINTS := 6
const MAX_REACH := 22.0
const DOF_AXES := ["Y", "Z", "Z", "Y", "Z", "Z"]


var udp_peer := PacketPeerUDP.new()
var server_ip := "127.0.0.1"
var server_port := 5005

@onready var camera = $CameraPivot/Camera3D
@onready var pivot = $CameraPivot
@onready var algo_dropdown = $UI/OptionButton

var renderer_mode: RendererMode = RendererMode.STICK

var joints_3d: Array[Node3D] = []
var bone_mesh_instance := MeshInstance3D.new()
var bone_mesh := ImmediateMesh.new()
var grid_mesh_instance := MeshInstance3D.new()
var grid_mesh := ImmediateMesh.new()

var target_height: float = 0.0

var mode_label: Label

var robot_arm_root: Node3D
var robot_joints: Array[Node3D] = []
var robot_fbx_parts: Array[Node3D] = []

var target_handle: MeshInstance3D
var is_dragging: bool = false

# Manual joint control
const JOINT_STEP := 0.1
const MANUAL_RATE := 3.0
var manual_angles: Array[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
var last_solver_angles: Array[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
var selected_joint: int = 0
var joint_info_label: Label
var manual_override: bool = false

func _ready():
	udp_peer.connect_to_host(server_ip, server_port)

	bone_mesh_instance.mesh = bone_mesh
	var bone_mat = StandardMaterial3D.new()
	bone_mat.albedo_color = Color.WHITE
	bone_mat.emission_enabled = true
	bone_mat.emission = Color.WHITE
	bone_mesh_instance.material_override = bone_mat
	add_child(bone_mesh_instance)

	grid_mesh_instance.mesh = grid_mesh
	var grid_mat = StandardMaterial3D.new()
	grid_mat.vertex_color_use_as_albedo = true
	grid_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	grid_mesh_instance.material_override = grid_mat
	add_child(grid_mesh_instance)

	algo_dropdown.add_item("FABRIK")
	algo_dropdown.add_item("CCD")
	algo_dropdown.add_item("JACOBIAN")

	mode_label = Label.new()
	mode_label.position = Vector2(10, 50)
	mode_label.add_theme_color_override("font_color", Color.YELLOW)
	$UI.add_child(mode_label)
	update_mode_label()

	joint_info_label = Label.new()
	joint_info_label.position = Vector2(10, 80)
	joint_info_label.add_theme_color_override("font_color", Color.CYAN)
	$UI.add_child(joint_info_label)

	build_robot_arm()

	var handle_mat = StandardMaterial3D.new()
	handle_mat.albedo_color = Color(1, 0.5, 0)
	handle_mat.emission_enabled = true
	handle_mat.emission = Color(1, 0.5, 0)
	handle_mat.emission_energy_multiplier = 2.0

	var handle_mesh = SphereMesh.new()
	handle_mesh.radius = 0.3
	handle_mesh.height = 0.6
	target_handle = MeshInstance3D.new()
	target_handle.mesh = handle_mesh
	target_handle.material_override = handle_mat
	add_child(target_handle)

	draw_grid()
	update_joint_info_label()

func _process(delta):
	if manual_override:
		if Input.is_key_pressed(KEY_Q):
			apply_manual_adjust(-MANUAL_RATE * delta)
		if Input.is_key_pressed(KEY_W):
			apply_manual_adjust(MANUAL_RATE * delta)

	var target_3d = target_handle.global_position
	if is_dragging:
		var mouse_3d = get_3d_mouse_pos()
		if mouse_3d != Vector3.ZERO:
			if mouse_3d.length() > MAX_REACH * 1.2:
				mouse_3d = mouse_3d.normalized() * MAX_REACH * 1.2
			target_3d = mouse_3d
			target_handle.global_position = target_3d

	var selected_algo = algo_dropdown.get_item_text(algo_dropdown.selected)

	var mode_str = "ROBOT" if renderer_mode == RendererMode.ROBOT else "3D"
	var data_to_send = {
		"target_pos": [target_3d.x, target_3d.y, target_3d.z],
		"algo": selected_algo,
		"mode": mode_str
	}
	udp_peer.put_packet(JSON.stringify(data_to_send).to_utf8_buffer())

	if udp_peer.get_available_packet_count() > 0:
		var response_string = udp_peer.get_packet().get_string_from_utf8()
		var json = JSON.new()
		if json.parse(response_string) == OK:
			var data_received = json.data
			if data_received.has("positions"):
				var positions = data_received["positions"]
				var angles = data_received.get("angles", [])
				draw_arm(positions, angles)

func draw_arm(positions: Array, angles: Array):
	match renderer_mode:
		RendererMode.STICK:
			draw_stick_arm(positions)
		RendererMode.ROBOT:
			draw_robot_arm(positions, angles)

func draw_stick_arm(positions: Array):
	if joints_3d.size() < positions.size():
		for i in range(positions.size() - joints_3d.size()):
			var new_joint = MeshInstance3D.new()
			var sphere = SphereMesh.new()
			sphere.radius = 0.2
			sphere.height = 0.4
			new_joint.mesh = sphere

			var mat = StandardMaterial3D.new()
			mat.albedo_color = Color.CYAN
			mat.emission_enabled = true
			mat.emission = Color.CYAN
			new_joint.material_override = mat

			add_child(new_joint)
			joints_3d.append(new_joint)

	for i in range(positions.size()):
		var pos = positions[i]
		joints_3d[i].position = Vector3(pos[0], pos[1], pos[2])

	bone_mesh.clear_surfaces()
	bone_mesh.surface_begin(Mesh.PRIMITIVE_LINES)
	for i in range(positions.size() - 1):
		bone_mesh.surface_add_vertex(Vector3(positions[i][0], positions[i][1], positions[i][2]))
		bone_mesh.surface_add_vertex(Vector3(positions[i+1][0], positions[i+1][1], positions[i+1][2]))
	bone_mesh.surface_end()

func build_robot_arm():
	robot_arm_root = Node3D.new()
	robot_arm_root.name = "RobotArm"
	add_child(robot_arm_root)

	var base = FBX_BASE.instantiate()
	base.scale = Vector3.ONE * MODEL_SCALE
	robot_arm_root.add_child(base)
	robot_fbx_parts.append(base)

	var joint0 = Node3D.new()
	joint0.name = "Joint0"
	joint0.position.y = 0.5 
	robot_arm_root.add_child(joint0)
	robot_joints.append(joint0)

	var base_rot = FBX_BASE_ROTATION.instantiate()
	base_rot.scale = Vector3.ONE * MODEL_SCALE
	base_rot.position.y = 0.0 
	base_rot.rotation = Vector3.ZERO
	joint0.add_child(base_rot)
	robot_fbx_parts.append(base_rot)
	
	var joint1 = Node3D.new()
	joint1.name = "Joint1"
	joint1.position.y = 1.2
	joint0.add_child(joint1)
	robot_joints.append(joint1)
	
	var arm1 = FBX_ARM_1.instantiate()
	arm1.scale = Vector3.ONE * MODEL_SCALE
	arm1.rotation = Vector3(0, PI, 0)
	joint1.add_child(arm1)
	robot_fbx_parts.append(arm1)
	
	var joint2 = Node3D.new()
	joint2.name = "Joint2"
	joint2.position.y = 3.1
	joint1.add_child(joint2)
	robot_joints.append(joint2)
	
	var h_joint = FBX_H_JOINT_BASE.instantiate()
	h_joint.scale = Vector3.ONE * MODEL_SCALE
	h_joint.position.y = 0.0
	h_joint.rotation = Vector3.ZERO
	joint2.add_child(h_joint)
	robot_fbx_parts.append(h_joint)

	var joint3 = Node3D.new()
	joint3.name = "Joint3"
	joint3.position.y = 0.85
	joint2.add_child(joint3)
	robot_joints.append(joint3)
	
	var h_recept = FBX_H_JOINT_RECEPTICLE.instantiate()
	h_recept.scale = Vector3.ONE * MODEL_SCALE
	h_recept.position.y = 0.0 
	h_recept.rotation = Vector3.ZERO
	joint3.add_child(h_recept)
	robot_fbx_parts.append(h_recept)

	var joint4 = Node3D.new()
	joint4.name = "Joint4"
	joint4.position.y = 1.3
	joint3.add_child(joint4)
	robot_joints.append(joint4)
	
	var arm2 = FBX_ARM_2.instantiate()
	arm2.scale = Vector3.ONE * MODEL_SCALE
	arm2.rotation = Vector3(0, PI, 0)
	joint4.add_child(arm2)
	robot_fbx_parts.append(arm2)

	var joint5 = Node3D.new()
	joint5.name = "Joint5"
	joint5.position.y = 5.25
	joint4.add_child(joint5)
	robot_joints.append(joint5)
	
	var h_joint2 = FBX_H_JOINT_BASE.instantiate()
	h_joint2.scale = Vector3.ONE * MODEL_SCALE
	h_joint2.position.y = 0.0
	h_joint2.rotation = Vector3.ZERO
	joint5.add_child(h_joint2)
	robot_fbx_parts.append(h_joint2)

	var endpoint_node = Node3D.new()
	endpoint_node.name = "Endpoint"
	endpoint_node.position.y = 0.825
	joint5.add_child(endpoint_node)
	
	var endpoint_mesh = FBX_ENDPOINT.instantiate()
	endpoint_mesh.scale = Vector3.ONE * MODEL_SCALE
	endpoint_mesh.position.y = 0.0 
	endpoint_mesh.rotation = Vector3.ZERO
	endpoint_node.add_child(endpoint_mesh)
	robot_fbx_parts.append(endpoint_mesh)

	for part in robot_fbx_parts:
		apply_glow(part, Color.ORANGE)

	robot_arm_root.visible = (renderer_mode == RendererMode.ROBOT)

func draw_robot_arm(positions: Array, angles: Array):
	if positions.size() < 7:
		return

	robot_arm_root.visible = true

	robot_arm_root.global_position = Vector3(positions[0][0], positions[0][1], positions[0][2])

	for i in range(min(robot_joints.size(), ACTIVE_JOINTS)):
		var angle: float
		if i < manual_angles.size() and manual_override:
			angle = manual_angles[i]
		elif i < angles.size():
			angle = angles[i]
			if i < last_solver_angles.size():
				last_solver_angles[i] = angle
			if i < manual_angles.size():
				manual_angles[i] = angle
		else:
			continue
		match DOF_AXES[i] if i < DOF_AXES.size() else "Z":
			"Y":
				robot_joints[i].rotation.y = angle
			"X":
				robot_joints[i].rotation.x = angle
			"Z":
				robot_joints[i].rotation.z = angle
			_:
				robot_joints[i].rotation = Vector3.ZERO
	update_joint_info_label()

func get_3d_mouse_pos() -> Vector3:
	var mouse_pos = get_viewport().get_mouse_position()
	var from = camera.project_ray_origin(mouse_pos)
	var to = from + camera.project_ray_normal(mouse_pos) * 1000

	var plane = Plane(Vector3.UP, target_height)
	var target_intersect = plane.intersects_ray(from, to)

	if target_intersect:
		return target_intersect
	return Vector3.ZERO

func draw_grid():
	grid_mesh.clear_surfaces()
	grid_mesh.surface_begin(Mesh.PRIMITIVE_LINES)

	var size = 100
	var step = 5
	var y_level = 0.01

	for i in range(-size, size + step, step):
		grid_mesh.surface_set_color(Color(0.3, 0.3, 0.3, 0.5))

		grid_mesh.surface_add_vertex(Vector3(i, y_level, -size))
		grid_mesh.surface_add_vertex(Vector3(i, y_level, size))

		grid_mesh.surface_add_vertex(Vector3(-size, y_level, i))
		grid_mesh.surface_add_vertex(Vector3(size, y_level, i))

	grid_mesh.surface_set_color(Color.RED)
	grid_mesh.surface_add_vertex(Vector3(-size, y_level, 0))
	grid_mesh.surface_add_vertex(Vector3(size, y_level, 0))

	grid_mesh.surface_set_color(Color.DODGER_BLUE)
	grid_mesh.surface_add_vertex(Vector3(0, y_level, -size))
	grid_mesh.surface_add_vertex(Vector3(0, y_level, size))

	grid_mesh.surface_set_color(Color.GREEN)
	grid_mesh.surface_add_vertex(Vector3(0, 0, 0))
	grid_mesh.surface_add_vertex(Vector3(0, size/4.0, 0))

	grid_mesh.surface_end()

func _input(event):
	if event is InputEventMouseMotion and Input.is_mouse_button_pressed(MOUSE_BUTTON_RIGHT):
		pivot.rotation.y -= event.relative.x * 0.005
		pivot.rotation.x -= event.relative.y * 0.005
		pivot.rotation.x = clamp(pivot.rotation.x, -PI/4, PI/4)

	if event is InputEventMouseButton:
		if event.button_index == MOUSE_BUTTON_LEFT:
			if event.pressed:
				var mouse_3d = get_3d_mouse_pos()
				if mouse_3d != Vector3.ZERO:
					target_handle.global_position = mouse_3d
					is_dragging = true
			else:
				is_dragging = false
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			target_height += 1.0
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			target_height -= 1.0
		target_height = clamp(target_height, 0.0, camera.global_position.y - 2.0)
	if event is InputEventKey and event.pressed and not event.echo:
		match event.keycode:
			KEY_V:
				toggle_renderer()
			KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6:
				var idx = event.keycode - KEY_1
				if idx < ACTIVE_JOINTS:
					selected_joint = idx
					manual_override = true
			KEY_R:
				manual_override = false
				for i in range(ACTIVE_JOINTS):
					if i < last_solver_angles.size():
						manual_angles[i] = last_solver_angles[i]

func toggle_renderer():
	renderer_mode = RendererMode.STICK if renderer_mode == RendererMode.ROBOT else RendererMode.ROBOT
	clear_arm()
	update_mode_label()

	target_handle.global_position = Vector3(5, 0, 5)
	is_dragging = false
	manual_override = false
	for i in range(ACTIVE_JOINTS):
		manual_angles[i] = 0.0
	selected_joint = 0

	var selected_algo = algo_dropdown.get_item_text(algo_dropdown.selected)
	var mode_str = "ROBOT" if renderer_mode == RendererMode.ROBOT else "3D"
	var data_to_send = {
		"target_pos": [5, 0, 5],
		"algo": selected_algo,
		"mode": mode_str,
		"reset": true
	}
	udp_peer.put_packet(JSON.stringify(data_to_send).to_utf8_buffer())

func clear_arm():
	for j in joints_3d:
		j.queue_free()
	joints_3d.clear()
	bone_mesh.clear_surfaces()
	if robot_arm_root:
		robot_arm_root.visible = (renderer_mode == RendererMode.ROBOT)


func update_mode_label():
	var names = {
		RendererMode.STICK: "STICK ARM [V]",
		RendererMode.ROBOT: "ROBOT ARM [V]"
	}
	mode_label.text = "Mode: " + names[renderer_mode]

func update_joint_info_label():
	var mode_str = "MANUAL" if manual_override else "IK"
	var names = ["J0(Y)", "J1(Z)", "J2(Z)", "J3(Y)", "J4(Z)", "J5(Z)"]
	var sel = selected_joint
	joint_info_label.text = "Mode: %s | Selected: %s [%d] = %.2f rad" % [mode_str, names[sel], sel, manual_angles[sel]]

func apply_manual_adjust(delta: float):
	manual_override = true
	manual_angles[selected_joint] += delta
	if selected_joint < len(DOF_AXES) and DOF_AXES[selected_joint] == "Z":
		manual_angles[selected_joint] = clamp(manual_angles[selected_joint], -1.57, 1.57)

func apply_glow(node: Node, color: Color):
	for child in node.get_children():
		if child is MeshInstance3D:
			var mat = StandardMaterial3D.new()
			mat.albedo_color = color
			mat.metallic = 0.7
			mat.roughness = 0.3
			mat.cull_mode = BaseMaterial3D.CULL_DISABLED
			
			mat.emission_enabled = false
			mat.emission = color
			mat.emission_energy_multiplier = 1.5
			
			child.material_override = mat
		apply_glow(child, color)
