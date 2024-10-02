# server.py
import socket

# Set up the server
HOST = '0.0.0.0'  # Listen on all available network interfaces
PORT = 65432      # Port to listen on (use a non-privileged port)

# Create a TCP/IP socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)  # Listen for incoming connections

print("Server listening on port", PORT)

# Accept a connection
conn, addr = server_socket.accept()
print('Connected by', addr)

try:
    while True:
        data = conn.recv(1024)  # Receive data in chunks
        if not data:
            break
        print("Received:", data.decode('utf-8'))
finally:
    # Clean up the connection
    conn.close()
