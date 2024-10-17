# server.py
import socket

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('192.168.4.1', 8080))  # Replace with your Raspberry Pi IP
server_socket.listen(1)

print("Waiting for a connection...")
conn, addr = server_socket.accept()
print(f"Connected to {addr}")

while True:
    data = conn.recv(1024).decode()
    if not data:
        break
    print(f"Received: {data}")

conn.close()
