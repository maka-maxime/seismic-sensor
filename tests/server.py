#!python
import socket

class Server:
    def __init__(self, port):
        self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__socket.settimeout(2)
        self.__socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.__socket.bind(('0.0.0.0', port))
        self.__service = None
        self.peer = None
    
    def listen(self, backlog=1):
        self.__socket.listen(backlog)
        self.__service, self.peer = self.__socket.accept()
    
    def receive(self) -> bytes:
        if self.__service is None:
            raise ConnectionError('No peer connected to server.')
        return self.__service.recv(1024)
        
    def send(self, data):
        if self.__service is None:
            raise ConnectionError('No peer connected to server.')
        self.__service.sendall(data)

    def close(self):
        if self.__service is not None:
            self.__service.close()
  
if __name__ == "__main__":
    server = Server(12345)
    data = None
    try:
        while True:
            try:
                print('Waiting for a connection...')
                server.listen()
                print(f'Peer found: {server.peer}')
                try:
                    data = server.receive()
                    print(f'Data received: {data.decode('utf-8')}')                      
                except Exception:
                    print('Exception')
            except socket.timeout:
                print('Socket timeout')
    except KeyboardInterrupt:
        print('Interrupted.')
        server.close()