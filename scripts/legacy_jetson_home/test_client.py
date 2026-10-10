import socket

HOST = "172.31.23.27"
PORT = 5000

client = socket.socket(
    socket.AF_INET,
    socket.SOCK_STREAM
)

client.connect((HOST, PORT))

client.send(
    b"start_mission"
)

reply = client.recv(1024)

print(reply.decode())

client.close()
