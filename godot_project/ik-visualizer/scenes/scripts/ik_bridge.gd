extends Node2D

var udp_peer := PacketPeerUDP.new()
var server_ip := "127.0.0.1"
var server_port := 5005

@onready var arm_line = $Line2D 

func _ready():
	udp_peer.connect_to_host(server_ip, server_port)
	RenderingServer.set_default_clear_color(Color(0.1, 0.1, 0.12))
	
	arm_line.width = 15.0
	arm_line.default_color = Color.CYAN
	
	arm_line.joint_mode = Line2D.LINE_JOINT_ROUND
	arm_line.begin_cap_mode = Line2D.LINE_CAP_ROUND
	arm_line.end_cap_mode = Line2D.LINE_CAP_ROUND
	
func _process(_delta):
	var mouse_pos = get_global_mouse_position()
	var data_to_send = {"target_pos": [mouse_pos.x, mouse_pos.y, 0.0]}
	
	udp_peer.put_packet(JSON.stringify(data_to_send).to_utf8_buffer())
	
	if udp_peer.get_available_packet_count() > 0:
		var response_string = udp_peer.get_packet().get_string_from_utf8()
		var json = JSON.new()
		if json.parse(response_string) == OK:
			var data_received = json.data
			if data_received.has("positions"): 
				draw_arm(data_received["positions"])

	queue_redraw()

func draw_arm(positions: Array):
	arm_line.clear_points()
	for pos in positions:
		arm_line.add_point(Vector2(pos[0], pos[1]))
		
func _draw():
	var current_mouse = get_global_mouse_position()
	draw_circle(current_mouse, 12.0, Color.CRIMSON)
	draw_circle(current_mouse, 6.0, Color.WHITE)
