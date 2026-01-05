#!python
import sys
import socket
import json
from ipaddress import ip_address, IPv4Address
from message import Message

class Client:
    def __init__(self, ip, port):
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.settimeout(2)
        self.__socket.connect((ip, port))

    def local_port(self):
        return self.__socket.getsockname()[1]

    def send(self, data):
        self.__socket.sendall(data)

    def receive(self) -> bytes:
        return self.__socket.recv(1024)

    def close(self):
        self.__socket.close()

if __name__ == "__main__":
    try:
        if len(sys.argv) != 4:
            print('usage: client.py <server_ipv4> <server_port> <message_type>')
            sys.exit(1)

        server_ip = sys.argv[1]

        try:
            if type(ip_address(server_ip)) is not IPv4Address:
                print('IPv6 is not supported.')
                sys.exit(1)
        except ValueError:
            print(f'error: {server_ip} is not a valid IPv4 address')
            sys.exit(1)

        if not sys.argv[2].isdigit():
            print(f'error: {sys.argv[2]} is not a valid TCP port number (1 to 65535)')
            sys.exit(1)
        
        server_port = int(sys.argv[2])

        message = None
        message_type = sys.argv[3].upper()

        if message_type == 'DATA':
            message = Message.DATA
        elif message_type == 'STATUS':
            message = Message.STATUS
        elif message_type == 'SYNC':
            message = Message.SYNC
        elif message_type == 'PRESENCE':
            message = Message.PRESENCE
        elif message_type == 'OKAY':
            message = Message.OKAY
        else:
            print('Message type invalid. Must be one of: DATA, STATUS, SYNC, PRESENCE, OKAY')
            sys.exit(1)

        client = Client(server_ip, server_port)
        print(f'Connected to {server_ip}:{server_port} through local port {client.local_port()}.')

        client.send(json.dumps(message, separators=(',',':')).encode('utf-8'))
        try:
            response = client.receive()
            if response:
                print(f'Message received: {response.decode('utf-8')}')
        except socket.timeout:
            print('Unable to receive a response.')

        client.close()
        sys.exit(0)
    except Exception:
        print('Execption handled.')
        sys.exit(1)