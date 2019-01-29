import socket
import sys
from thread import start_new_thread

def clientHandler(cli):
    
    while True:
        data = cli.recv(1024).decode()
        if not data:
        # if data is not received break
            break

        print("from connected user: " + str(data))

    cli.close()  # close the connection

def server_program():
    # host = socket.gethostname()
    host =socket.gethostbyname(socket.getfqdn())
    # define host manually
    host='10.157.137.49'
    print(host)
    port = 4999  # initiate port no above 1024

    try:
        server_socket = socket.socket()  # get instance
    except socket.error,msg:
        print("Could not create socket. Error code:", str(msg[0]), "Error: ", msg[1])
        sys.exit(0)
    print("server is created")
    # look closely. The bind() function takes tuple as argument
    try:
        server_socket.bind((host, port))  # bind host address and port together
    except socket.error,msg:
        print("bind failted. Error Code: {} Error: {}", format(str(msg[0]),msg[1]))
        sys.exit(0)

    # configure how many client the server can listen simultaneously
    server_socket.listen(3)
    print("Waiting...")
    
    while True:
        conn,address=server_socket.accept()
        print("Connection from: " + str(address))
        #start new thread 
        start_new_thread(clientHandler, (conn,))

    sever_socket.close()  # close the connection


if __name__ == '__main__':
    server_program()

