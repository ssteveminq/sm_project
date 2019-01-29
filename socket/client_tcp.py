import socket
from platform import python_version


#input(): for above python 3.x
#raw_input(): for above python 2.x

def client_program():
    # host = socket.gethostname()  # as both code is running on same pc
    host = '10.157.137.49'# as both code is running on same pc
    port = 4998  # socket server port number

    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server

    version_int= int(python_version()[0])
    if(version_int>0):
        message = input(' -> ')  # take input
    else:
        message = raw_input(' -> ')  # take input


    # while message.lower().strip() != 'bye':
    while message.lower() != 'bye':
        client_socket.send(message.encode())  # send message
        # client_socket.send(message)  # send message
        # data = client_socket.recv(1024).decode()  # receive response

        # print('Received from server: ' + data)  # show in terminal
        message = input(" -> ")  # again take input
        if(version_int>0):
            message = input(' -> ')  # take input
        else:
            message = raw_input(' -> ')  # take input

    client_socket.close()  # close the connection


if __name__ == '__main__':
    client_program()
