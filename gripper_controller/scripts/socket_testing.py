import sys
import socket

HOST = "127.0.0.1" # The UR IP address
PORT = 30002 # UR secondary client
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.bind((HOST, PORT))
sock.listen()
conn, addr = sock.accept()
with conn:
    print(f"Connected by {addr}")
    while True:
        data = conn.recv(2024)
        if not data:
            break
        print(data)