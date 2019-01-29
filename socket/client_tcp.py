import socket


#input(): for above python 3.x
#raw_input(): for above python 2.x

def client_program():
    # host = socket.gethostname()  # as both code is running on same pc
    host = '10.157.137.49'  # as both code is running on same pc
    port = 4999  # socket server port number

    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server

    message = raw_input(' -> ')  # take input

    # while message.lower().strip() != 'bye':
    while message.lower() != 'bye':
        client_socket.send(message.encode())  # send message
        # client_socket.send(message)  # send message
        # data = client_socket.recv(1024).decode()  # receive response

        # print('Received from server: ' + data)  # show in terminal
        message = raw_input(" -> ")  # again take input

    client_socket.close()  # close the connection


if __name__ == '__main__':
    client_program()
