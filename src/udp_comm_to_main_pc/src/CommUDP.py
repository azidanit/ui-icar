
import socket
from threading import Thread, Lock

import os

import struct, time

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8, UInt8MultiArray, Int32MultiArray, Float32
'''
1 -> SAMPAI TERMINAL / uybah keberangkatan
2 -> Berangkat
3 -> 
'''
class CommUDP:
    def __init__(self) -> None:
        self.main_pc_ip = "192.168.50.254"
        self.main_pc_port_rx = 9002
        self.main_pc_port_tx = 9001

        self.bufferSize = 1024        

        self.serverAddressPort = (self.main_pc_ip, self.main_pc_port_tx)

        self.publish_pos = rospy.Publisher('/mavros/global_position/global', NavSatFix, queue_size=10)
        self.state_terminal_pub = rospy.Publisher("/autonomous_car/state_terminal", Int32MultiArray,queue_size=10)
        self.eta_pub = rospy.Publisher("/user_interface/eta", Float32,queue_size=10)

        self.terminal_cmd_sub = rospy.Subscriber("/webapi/terminal_call", Int8, callback=self.terminalCallCallback)
        self.ui_cmd_sub = rospy.Subscriber("/user_interface/user_cmd", UInt8MultiArray, callback=self.uiCallCallback)

        self.is_arrived = False

        self.status_auto = 0
        self.terminal_cmd = 0
        self.going_to_terminal = 0
        self.going_to_terminal_before = -1

        self.var_mtx = Lock()

        # self.initUdpSend()
        self.initUdpReceive()

        self.dist_before = 0
        self.is_car_start = 0
        self.lat = -7.277766
        self.long = 112.797430
        pass

        self.start()
        # self.openLastFile()

    def openLastFile(self):
        dir = "last_position.txt"
        print("OPENING LAST FILE")
        if not os.path.isfile(dir):
            print("FILE NOT FOUND, making new file")
            self.writeFile()
        else:
            print("Opening file")
            f = open(dir, "r")
            print(f.read())
        pass

    def writeFile(self):
        dir = "last_position.xt"
        print("writing file")
        f = open(dir, "w")
        str_msg = "{},{}".format(self.lat, self.long)
        f.write(str_msg)
        f.close()

    def start(self):
        # self.listenUDPThread()
        # self.listen_thread = Thread(target=self.listenUDPThread)
        # self.listen_thread.start()

        self.send_thread = Thread(target=self.sendUDPThread)
        self.send_thread.start()

    def initUdpSend(self):
        try:
            self.UDPServerSocket_tx = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            self.UDPServerSocket_tx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.UDPServerSocket_tx.settimeout(2)
        except Exception as e:
            print(e)
            self.initUdpSend()

    def initUdpReceive(self):
        # Create a datagram socket
        try:
            self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
            self.UDPServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.UDPServerSocket.settimeout(2)
        except Exception as e:
            print(e)
            self.initUdpReceive()


    def sendTerminalCmd(self):
        pass

    def listenUDPThread(self):
        bufferSize  = 1024

        # Bind to address and ip

        self.UDPServerSocket.bind(('0.0.0.0', self.main_pc_port_rx))

        

        print("UDP server up and listening")

        

        # Listen for incoming datagrams

        while not rospy.is_shutdown():
            print("waiting udp msg")
            try:
                bytesAddressPair = self.UDPServerSocket.recvfrom(bufferSize)

                message = bytesAddressPair[0]

                address = bytesAddressPair[1]
                self.parseIncomingMsg(message)
            except Exception as e:
                print(e)
            

    def parseIncomingMsg(self, msg):
        if len(msg)<25:
            return
        # spliited_msg = msg.split['/']
        lat_ = msg[:8]
        long_ = msg[8:16]
        dist_ = msg[16:20]
        eta_ = msg[20:24]
        is_start = msg[24:25]
        terminal_go = msg[25:26]

        lat_ = struct.unpack('d', lat_)[0]
        long_ = struct.unpack('d', long_)[0]
        dist_ = struct.unpack('f', dist_)[0]
        eta_ = struct.unpack('f', eta_)[0]
        is_start = struct.unpack('b',is_start)[0]
        terminal_go = struct.unpack('b',terminal_go)[0]

        print(lat_, long_, dist_, eta_, terminal_go)

        self.lat = lat_
        self.long = long_

# 
        nav_sat_msg = NavSatFix()
        nav_sat_msg.latitude = lat_
        nav_sat_msg.longitude = long_

        self.publish_pos.publish(nav_sat_msg)
        self.eta_pub.publish(eta_)

        # if float(dist_) <= 3:
        #     if not self.is_arrived:
        #         msg_int32 = Int32MultiArray()
        #         msg_int32.data.append(self.going_to_terminal)
        #         msg_int32.data.append(1)
        #         self.state_terminal_pub.publish(msg_int32)
        #         msg_int32 = Int32MultiArray()
        #         msg_int32.data.append(self.going_to_terminal)
        #         msg_int32.data.append(4)
        #         self.state_terminal_pub.publish(msg_int32)

        #         self.var_mtx.acquire()
        #         self.status_auto = 0
        #         self.terminal_cmd = 0
        #         self.var_mtx.release()

        #         self.is_arrived = True
        # else:
        #     self.is_arrived = False            
        # pass

        if is_start != self.is_car_start:
            if is_start == 1:
                if self.status_auto != 1:
                    #doing play sound
                    msg_int32 = Int32MultiArray()
                    msg_int32.data.append(0)
                    msg_int32.data.append(2)
                    self.state_terminal_pub.publish(msg_int32)
                pass
            elif is_start == 0:
                msg_int32 = Int32MultiArray()
                msg_int32.data.append(self.going_to_terminal)
                msg_int32.data.append(1)
                self.state_terminal_pub.publish(msg_int32)
                msg_int32 = Int32MultiArray()
                msg_int32.data.append(self.going_to_terminal)
                msg_int32.data.append(4)
                self.state_terminal_pub.publish(msg_int32)


        if terminal_go != self.going_to_terminal :
            msg_int32 = Int32MultiArray()
            msg_int32.data.append(terminal_go + 1)
            msg_int32.data.append(5)
            self.state_terminal_pub.publish(msg_int32)

            self.going_to_terminal = terminal_go
        
        print("DISTANCE ", dist_, self.dist_before)
        print("IS START ", is_start, self.is_car_start)

        self.is_car_start = is_start
        self.dist_before = dist_



    def sendUDPThread(self):
        while not rospy.is_shutdown():
            try:
                print("GOING TO TERMINAL", self.going_to_terminal)
                bytesToSend = bytearray(struct.pack("b", self.status_auto))
                bytesToSend += bytearray(struct.pack("b", self.terminal_cmd))
                print("sending", bytesToSend, len(bytesToSend))
                self.UDPServerSocket.sendto(bytesToSend, self.serverAddressPort)
                try:
                    bytesAddressPair = self.UDPServerSocket.recvfrom(self.bufferSize)

                    message = bytesAddressPair[0]

                    address = bytesAddressPair[1]
                    self.parseIncomingMsg(message)
                except Exception as e:
                    print(e)
            except Exception as e:
                print("ERROR", e)
            time.sleep(0.5)

    def terminalCallCallback(self, msg):
        self.var_mtx.acquire()
        self.status_auto = 1
        self.terminal_cmd = msg.data
        self.going_to_terminal= msg.data
        self.var_mtx.release()

        msg_int32 = Int32MultiArray()
        msg_int32.data.append(self.going_to_terminal)
        msg_int32.data.append(2)
        self.state_terminal_pub.publish(msg_int32)


    def uiCallCallback(self, msg):
        if len(msg.data)>1:
            self.var_mtx.acquire()
            self.status_auto = msg.data[0]
            self.terminal_cmd = msg.data[1]
            self.going_to_terminal= self.terminal_cmd
            self.var_mtx.release()

            if self.status_auto == 1:
                msg_int32 = Int32MultiArray()
                msg_int32.data.append(self.going_to_terminal)
                msg_int32.data.append(2)
                self.state_terminal_pub.publish(msg_int32)
        
