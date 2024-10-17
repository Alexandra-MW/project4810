import socket

# Set up the client to connect to the MacBook server
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.2.1', 8080))  # Use your MacBook's IP address

while True:
    data = client_socket.recv(1024).decode()  # Receive the positional data
    if not data:
        break
    # Print or process the received data
    print(f"Received positional data: {data}")

client_socket.close()
