#!/usr/bin/env python3

import socket

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = ????         # The port used by the server

# Open the connection (FicTrac must be waiting for socket connection)
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((HOST, PORT))
    
    # Keep receiving data until FicTrac closes
    while True:
        # Receive one data frame
        data = sock.recv(1024)
        if not data:
            break
        
        # Decode received data
        line = data.decode('UTF-8')
        endline = line.find("\n")
        line = line[:endline]
        toks = line.split(", ")
        
        # Fixme: sometimes we read more than one line at a time,
        # should handle that rather than just dropping extra data...
        if ((len(toks) < 24) | (toks[0] != "FT")):
            print('Bad read')
            continue
        
        # Extract FicTrac variables
        # (see https://github.com/rjdmoore/fictrac/blob/master/doc/data_header.txt for descriptions)
        cnt = int(toks[1])
        dr_cam = [float(toks[2]), float(toks[3]), float(toks[4])]
        err = float(toks[5])
        dr_lab = [float(toks[6]), float(toks[7]), float(toks[8])]
        r_cam = [float(toks[9]), float(toks[10]), float(toks[11])]
        r_lab = [float(toks[12]), float(toks[13]), float(toks[14])]
        posx = float(toks[15])
        posy = float(toks[16])
        heading = float(toks[17])
        step_dir = float(toks[18])
        step_mag = float(toks[19])
        intx = float(toks[20])
        inty = float(toks[21])
        ts = float(toks[22])
        seq = int(toks[23])
        
        # Do something ...
        print(cnt)
