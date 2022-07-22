import os
import sys
import socket

HOST = "192.168.0.100" # The UR IP address
PORT = 30002 # UR secondary client
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sock.settimeout(0.5)
try: sock.connect((HOST, PORT))
except:
    raise Exception("Cannot connect to end-effector socket") from None
sock.settimeout(None)

scripts_path = os.path.dirname(os.path.realpath(__file__))
onrobot_script = scripts_path + "/onrobot.script";
file = open(onrobot_script, "rb")   # Robotiq Gripper

lines = file.readlines()
file.close()

diameter = 25
external_release = True
tool_index = 0
select_releasing = 0
blocking = True
if len(sys.argv):# >= 5:
    diameter = sys.argv[1]
    # external_release = sys.argv[1]
    # tool_index = sys.argv[2]
    # select_releasing = sys.argv[3]
    # blocking = sys.argv[4]

cmd_string = f"tfg_release({diameter},  tool_index={tool_index}, blocking={blocking})"

line_number_to_add = 1289

new_lines = lines[0:line_number_to_add]
new_lines.insert(line_number_to_add+1, str.encode(cmd_string))
new_lines += lines[line_number_to_add::]

offset = 0
buffer = 2024
file_to_send = b''.join(new_lines)

if len(file_to_send) < buffer:
    buffer = len(file_to_send)
data = file_to_send[0:buffer]
while data:
    # print(offset, buffer)
    # print(data)
    # print("---------------------------------")

    sock.send(data)
    offset += buffer
    if len(file_to_send) < offset+buffer:
        buffer = len(file_to_send) - offset
    data = file_to_send[offset:offset+buffer]

sock.close()

