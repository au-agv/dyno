import socket
import json

UDP_IP = "127.0.0.1"
UDP_PORT = 1111
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)

sock = socket.socket(
    socket.AF_INET,  # Internet
    socket.SOCK_DGRAM)  # UDP
sock.setblocking(1)
sock.connect((UDP_IP, UDP_PORT))

while True:
    try:
        mydict = {"hello": 123}
        sock.send(json.dumps(mydict, ensure_ascii=False).encode('gbk'))
        raw = sock.recv(1024)
        print(raw)
    except OSError as err:
        if err.errno == socket.EWOULDBLOCK:
            break
        raise
