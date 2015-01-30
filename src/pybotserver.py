#!/usr/bin/python2
# coding=UTF-8

import socket
import os

if __name__ == "__main__":
    host = ''
    port = 12345
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(1)
    conn, addr = s.accept()
    print('Connected by', addr)
    while True:
        data = conn.recv(1024)
        print data
        if data == 'exit':
            break
    conn.close()
