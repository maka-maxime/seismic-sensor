#!python
import json
import socket
import sys
from message import Message

class Broadcast:
  def __init__(self):
    self.__socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
    self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

  def sendto(self, data: bytes, port: int) -> int:
    return self.__socket.sendto(data, ('255.255.255.255', port))

  def close(self):
    self.__socket.close()

def main():
    if len(sys.argv) != 2:
        print('usage: broadcast.py <port_number>')
        sys.exit(1)
    
    if not sys.argv[1].isdigit():
        print(f'error: {sys.argv[1]} is not a valid UDP port number (1 to 65535)')
        sys.exit(1)
    
    port = int(sys.argv[1])

    broadcast = Broadcast()
    broadcast.sendto(json.dumps(Message.PRESENCE, separators=(',',':')).encode('utf-8'), port)
    broadcast.close()

if __name__ == '__main__':
    main()
