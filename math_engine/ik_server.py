import socket
import json

HOST = '127.0.0.1'
PORT = 5005

# Create a UDP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))

print(f"Python IK Server running on {HOST}:{PORT}...")
print("Waiting for Godot to send targets...\n")

while True:
    try:
        # Wait to receive data from Godot (buffer size 1024 bytes)
        data, address = server_socket.recvfrom(1024)
        
        # Decode the incoming JSON string
        message = json.loads(data.decode('utf-8'))
        print(f"Received from Godot: {message}")
        
        # Fake the math output to test the bridge
        target_x = message.get("target_pos", [0, 0, 0])[0]
        fake_angles = [target_x * 2.0, 45.0, 90.0] 
        # -----------------------------------
        
        # Package the angles into JSON and send them back
        response = {"angles": fake_angles}
        response_data = json.dumps(response).encode('utf-8')
        
        server_socket.sendto(response_data, address)
        
    except KeyboardInterrupt:
        print("\nShutting down server.")
        break
    except Exception as e:
        print(f"Error: {e}")