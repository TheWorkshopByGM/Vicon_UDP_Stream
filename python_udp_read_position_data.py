"""
Crazyflie 101 Tutorial - python_udp_read_position_data.py
Author: Guy Maalouf
Date: May 12, 2023

This code will only extract the position (x,y,z) and attitude data (pitch, roll, yaw) from the Vicon UDP stream
"""

import sys
import os
import time
import struct
import socket
import select
import numpy as np
from threading import Thread

class ViconUDPDataRelay(Thread):
    def __init__(self, RX_sock):
        Thread.__init__(self)
        self.DEBUG = False
        # Define Message length
        self.MsgLen = 1024
        self.RX_sock = RX_sock
        self.MessageRX_flag = False
        self.SrvMsgReceived = True
        self.RxMessage = None
        # Entry dictionary
        self.object_dict = {}
        self.reset_object_dict()
        
    def reset_object_dict(self):
        self.object_dict['number_objects'] = 0

    def ReceiveMsgOverUDP(self):
        Header = 'Waiting!'
        # Select RX_sock
        sock = self.RX_sock
        # Verify if data has been received from VICON
        ready = select.select([sock], [], [], 0.001)
        # If data has been received, process it
        if ready[0]:
            self.MessageRX_flag = True
            data, addr = sock.recvfrom(self.MsgLen)
            # Extract message header
            Sequence_B00 = data[0]
            Sequence_B01 = data[1]
            Sequence_B02 = data[2]
            Sequence_B03 = data[3]
            Sequence = (Sequence_B03<<32)+(Sequence_B02<<16)+(Sequence_B01<<8)+Sequence_B00
            self.ProcessViconData(data[0:160])

    def ProcessViconData(self, data):
        # Create struct with message format:
        # Data types according to https://docs.python.org/3/library/struct.html
        # FrameNumber                       -> B000 - B003 (uint32,  I)
        # ItemsInBlock                      -> B004        (uint8,   B)
        #              -----------------------------------------------
        #                      ItemID       -> B005        (uint8,   B)
        #              Header  ItemDataSize -> B006 - B007 (uint16,  H)
        #                      ItemName     -> B008 - B031 (uint8,   B)
        #              -----------------------------------------------
        # Item_raw_00          TransX       -> B032 - B039 (double,  d)
        #                      TransY       -> B040 - B047 (double,  d)
        #                      TransZ       -> B048 - B055 (double,  d)
        #              Data    RotX         -> B056 - B063 (double,  d)
        #                      RotY         -> B064 - B071 (double,  d)
        #                      RotZ         -> B072 - B079 (double,  d)    
        #              -----------------------------------------------
        #                      ItemID       -> B080        (uint8,   B)
        #              Header  ItemDataSize -> B081 - B082 (uint16,  H)
        #                      ItemName     -> B083 - B106 (uint8,   B)
        #              -----------------------------------------------
        # Item_raw_01          TransX       -> B107 - B114 (double,  d)
        #                      TransY       -> B115 - B122 (double,  d)
        #                      TransZ       -> B123 - B130 (double,  d)
        #              Data    RotX         -> B131 - B139 (double,  d)
        #                      RotY         -> B140 - B146 (double,  d)
        #                      RotZ         -> B147 - B154 (double,  d)    
        #              -----------------------------------------------
        s = struct.Struct('I2BH24c6dBH24c6d')
        UnpackedData = s.unpack(data)
        FrameNumber  = UnpackedData[0]
        ItemsInBlock = UnpackedData[1]
        Item_raw_00_ItemID       = UnpackedData[2]
        Item_raw_00_ItemDataSize = UnpackedData[3]
        Item_raw_00_ItemName     = UnpackedData[4:28]
        Item_raw_00_TransX       = UnpackedData[28]
        Item_raw_00_TransY       = UnpackedData[29]
        Item_raw_00_TransZ       = UnpackedData[30]
        Item_raw_00_RotX         = UnpackedData[31]
        Item_raw_00_RotY         = UnpackedData[32]
        Item_raw_00_RotZ         = UnpackedData[33]
        Item_raw_00_ItemDataSize_string = []
        for this_byte in range(0,len(Item_raw_00_ItemName)):
            if Item_raw_00_ItemName[this_byte]>= b'!' and Item_raw_00_ItemName[this_byte]<= b'~':
                Item_raw_00_ItemDataSize_string.append(Item_raw_00_ItemName[this_byte].decode('utf-8'))
        Item_raw_00_ItemDataSize_string = ''.join(Item_raw_00_ItemDataSize_string)
        self.object_dict[Item_raw_00_ItemDataSize_string] = {'PosX':Item_raw_00_TransX,'PosY':Item_raw_00_TransY,
                                                             'PosZ':Item_raw_00_TransZ,'RotX':Item_raw_00_RotX,
                                                             'RotY':Item_raw_00_RotY,'RotZ':Item_raw_00_RotZ}
        self.object_dict['number_objects'] += 1

        if self.DEBUG:
            print('\tPosition [cm]: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(self.object_dict[Item_raw_00_ItemDataSize_string]['PosX']*1e-1,
                                                                               self.object_dict[Item_raw_00_ItemDataSize_string]['PosY']*1e-1,
                                                                               self.object_dict[Item_raw_00_ItemDataSize_string]['PosZ']*1e-1))
            print('\tAttitude [deg]: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotX']),
                                                                           np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotY']),
                                                                           np.rad2deg(self.object_dict[Item_raw_00_ItemDataSize_string]['RotZ'])))
            print('----------------------------------')
        
    def close(self):
        pass
    
if __name__ == "__main__":
    IP_Address01 = "0.0.0.0"
    Host01   = (IP_Address01, 51001)
    # Create RX socket
    RX_sock = socket.socket(socket.AF_INET,    # Internet
                                     socket.SOCK_DGRAM) # UDP
    RX_sock.bind(Host01)
    MyViconDataRelay = ViconUDPDataRelay(RX_sock)
    MyViconDataRelay.DEBUG = True
    while True:
        try:
            MyViconDataRelay.ReceiveMsgOverUDP()
            
        except(KeyboardInterrupt,SystemExit):
            print("\nClosing program ...")
            # Close socket
            RX_sock.close()
            # Exit program
            sys.exit()
        
        except socket.error as msg:
            print("Socket error!")
            # Close socket
            RX_sock.close()
            print(msg)
            sys.exit()
