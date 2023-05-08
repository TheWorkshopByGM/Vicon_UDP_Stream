"""
Vicon UDP Stream - python_udp_read_raw_data.py
Author: Guy Maalouf
Date: May 12, 2023

This code is meant to read the Vicon UDP stream, and extract the data from within it.
"""

import sys
import struct
import socket
import select
import numpy as np
from threading import Thread

class ViconUDPDataRelay(Thread):
    def __init__(self, RX_sock):
        Thread.__init__(self)
        self.DEBUG = False
        self.MsgLen = 1024
        self.RX_sock = RX_sock
        self.MessageRX_flag = False
        self.SrvMsgReceived = True
        self.RxMessage = None
        self.object_dict = {}
        self.reset_object_dict()
        
    def reset_object_dict(self):
        self.object_dict['number_objects'] = 0

    def ReceiveMsgOverUDP(self):
        Header = 'Waiting!'
        sock = self.RX_sock
        ready = select.select([sock], [], [], 0.001)
        if ready[0]:
            self.MessageRX_flag = True
            data, addr = sock.recvfrom(self.MsgLen)
            Sequence_B00 = data[0]
            Sequence_B01 = data[1]
            Sequence_B02 = data[2]
            Sequence_B03 = data[3]
            Sequence = (Sequence_B03<<32)+(Sequence_B02<<16)+(Sequence_B01<<8)+Sequence_B00
            if self.DEBUG:
                print('\nVICON data received!!!')
                print('Header: {0}'.format(data[0:8]))
                print('Sequence: {0:2d}'.format(Sequence))
                print('byte 03: {0:2d}'.format(data[3]))
                print('byte 04: {0:2d}'.format(data[4]))
                print('byte 05: {0:2d}'.format(data[5]))
                print('byte 06: {0:2d}'.format(data[6]))
                print('byte 07: {0:2d}'.format(data[7]))
                print('Object ID string: {0}'.format(data[8:32]))
                print('Data length: {0}'.format(len(data)))
                print('Data B00: {0}'.format(data[32]))
                print('Data: {0}'.format(data[32:80+5]))
            #
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
        if self.DEBUG:
            print('Message content -> Frame number: {0}, items in block: {1}'.format(FrameNumber,ItemsInBlock))
            print('Item00 -> ID: {0}, Data size: {1}, Name: {2}'.format(Item_raw_00_ItemID,
                                                                        Item_raw_00_ItemDataSize,Item_raw_00_ItemDataSize_string))
            print('Item00 -> Position: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(Item_raw_00_TransX*1e-1,Item_raw_00_TransY*1e-1,
                                                                               Item_raw_00_TransZ*1e-1))
            print('Item00 -> Attitude: {0:+3.4f}, {1:+3.4f}, {2:+3.4f}'.format(np.rad2deg(Item_raw_00_RotX),np.rad2deg(Item_raw_00_RotY),
                                                             np.rad2deg(Item_raw_00_RotZ)))
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
        
    def close(self):
        pass

if __name__ == "__main__":
    IP_Address01 = "0.0.0.0"
    Host01   = (IP_Address01, 51001)
    RX_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    RX_sock.bind(Host01)
    MyViconDataRelay = ViconUDPDataRelay(RX_sock)
    MyViconDataRelay.DEBUG = True
    while True:
        try:
            MyViconDataRelay.ReceiveMsgOverUDP()
            
        except(KeyboardInterrupt,SystemExit):
            print("\nClosing program ...")
            RX_sock.close()
            sys.exit()
        
        except socket.error as msg:
            print("Socket error!")
            RX_sock.close()
            print(msg)
            sys.exit()
