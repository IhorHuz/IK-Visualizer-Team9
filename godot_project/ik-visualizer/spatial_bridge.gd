extends Node3D

var udp_peer := PacketPeerUDP.new()
var server_ip := "127.0.0.1"
var server_port := 5005

@onready var camera = $CameraPivot/Camera3D
@onready var pivot = $CameraPivot
@onready var algo_dropdown = $UI/OptionButton

var joints_3d: Array[Node3D] = []
var bone_mesh_instance := MeshInstance3D.new()
var bone_mesh := ImmediateMesh.new()
var grid_mesh_instance := MeshInstance3D.new()
var grid_mesh := ImmediateMesh.new()

# var base_rot_scene = preload("res://visuals/Base_rotation.fbx")
# var arm_1_scene = preload("res://visuals/arm_1.fbx")
# var arm_2_scene = preload("res://visuals/arm_2.fbx")
# var endpoint_scene = preload("res://visuals/endpoint.fbx")
# var base_scene = preload("res://visuals/Base.fbx")

var target_height: float = 0.0

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
	
	# var static_base = base_scene.instantiate()
	# static_base.scale = Vector3(0.01, 0.01, 0.01)
	# add_child(static_base)
	# apply_glow(static_base, Color.YELLOW)
	
	draw_grid()

func _process(_delta):
	var target_3d = get_3d_mouse_pos()
	
	var selected_algo = algo_dropdown.get_item_text(algo_dropdown.selected)
	
	var data_to_send = {
		"target_pos": [target_3d.x, target_3d.y, target_3d.z],
		"algo": selected_algo,
		"mode": "3D"
	}
	
	udp_peer.put_packet(JSON.stringify(data_to_send).to_utf8_buffer())
	
	if udp_peer.get_available_packet_count() > 0:
		var response_string = udp_peer.get_packet().get_string_from_utf8()
		var json = JSON.new()
		if json.parse(response_string) == OK:
			var data_received = json.data
			if data_received.has("positions"): 
				draw_arm(data_received["positions"])

func draw_arm(positions: Array):
	if joints_3d.size() < positions.size():
		for i in range(positions.size() - joints_3d.size()):
			var new_joint = MeshInstance3D.new()
			var sphere = SphereMesh.new()
			sphere.radius = 0.5
			sphere.height = 1.0
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
		if event.button_index == MOUSE_BUTTON_WHEEL_UP:
			target_height += 1.0
		elif event.button_index == MOUSE_BUTTON_WHEEL_DOWN:
			target_height -= 1.0
			
		target_height = clamp(target_height, 0.0, camera.global_position.y - 2.0)

func apply_glow(node: Node, color: Color):
	for child in node.get_children():
		if child is MeshInstance3D:
			var mat = StandardMaterial3D.new()

			mat.albedo_color = color

			mat.emission_enabled = false 

			mat.metallic = 0.8
			mat.roughness = 0.3 

			child.material_override = mat
			
		apply_glow(child, color)
