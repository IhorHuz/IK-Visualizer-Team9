extends Node2D

var udp_peer := PacketPeerUDP.new()
var server_ip := "127.0.0.1"
var server_port := 5005

@onready var arm_line = $Line2D 

var link_lengths = [150.0, 150.0, 150.0]

func _ready():
	udp_peer.connect_to_host(server_ip, server_port)
	print("Godot UDP Client started. Sending to ", server_ip, ":", server_port)

func _process(_delta):
	var mouse_pos = get_global_mouse_position()
	var data_to_send = {"target_pos": [mouse_pos.x, mouse_pos.y, 0.0]}
	
	var json_string = JSON.stringify(data_to_send)
	udp_peer.put_packet(json_string.to_utf8_buffer())
	
	if udp_peer.get_available_packet_count() > 0:
		var packet = udp_peer.get_packet()
		var response_string = packet.get_string_from_utf8()
		
		var json = JSON.new()
		var error = json.parse(response_string)
		if error == OK:
			var data_received = json.data
			if data_received.has("angles"):
				draw_arm(data_received["angles"])

func draw_arm(angles: Array):
	var current_pos = Vector2(500, 300) 
	var current_angle = 0.0
	
	arm_line.clear_points()
	arm_line.add_point(current_pos)
	
	for i in range(min(angles.size(), link_lengths.size())):
		current_angle += deg_to_rad(angles[i]) 
		
		var next_pos = current_pos + Vector2(cos(current_angle), sin(current_angle)) * link_lengths[i]
		arm_line.add_point(next_pos)
		
		current_pos = next_pos
