
from codecs import latin_1_decode
import socket
from threading import Thread, Lock

import struct, time

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8, UInt8MultiArray, Int32MultiArray

class CommUDP:
    def __init__(self) -> None:
        self.main_pc_ip = "192.168.50.251"
        self.main_pc_port_rx = 9002
        self.main_pc_port_tx = 9001

        self.bufferSize = 1024        

        self.serverAddressPort = (self.main_pc_ip, self.main_pc_port_tx)

        self.publish_pos = rospy.Publisher('/mavros/global_position/global', NavSatFix, queue_size=10)
        self.state_terminal_pub = rospy.Publisher("/autonomous_car/state_terminal", Int32MultiArray,queue_size=10)

        self.terminal_cmd_sub = rospy.Subscriber("/webapi/terminal_call", Int8, callback=self.terminalCallCallback)
        self.ui_cmd_sub = rospy.Subscriber("/user_interface/user_cmd", UInt8MultiArray, callback=self.uiCallCallback)

        self.is_arrived = False

        self.status_auto = 0
        self.terminal_cmd = 0

        self.var_mtx = Lock()

        self.initUdpSend()
        self.initUdpReceive()
        pass

    def start(self):
        # self.listenUDPThread()
        # self.listen_thread = Thread(target=self.listenUDPThread)
        # self.listen_thread.start()

        self.send_thread = Thread(target=self.sendUDPThread)
        self.send_thread.start()

    def initUdpSend(self):
        self.UDPServerSocket_tx = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket_tx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.UDPServerSocket_tx.settimeout(2)

    def initUdpReceive(self):
        # Create a datagram socket

        self.UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.UDPServerSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.UDPServerSocket.settimeout(2)
        

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
        if len(msg)<24:
            return
        # spliited_msg = msg.split['/']
        lat_ = msg[:8]
        long_ = msg[8:16]
        dist_ = msg[16:20]
        eta_ = msg[20:24]

        lat_ = struct.unpack('d', lat_)[0]
        long_ = struct.unpack('d', long_)[0]
        dist_ = struct.unpack('f', dist_)[0]
        eta_ = struct.unpack('f', eta_)[0]
        print(lat_, long_, dist_, eta_)

        nav_sat_msg = NavSatFix()
        nav_sat_msg.latitude = lat_
        nav_sat_msg.longitude = long_

        self.publish_pos.publish(nav_sat_msg)

        if float(dist_) <= 3:
            if not self.is_arrived:
                msg_int32 = Int32MultiArray()
                msg_int32.data.append(2)
                msg_int32.data.append(1)
                self.state_terminal_pub.publish(msg_int32)
                msg_int32 = Int32MultiArray()
                msg_int32.data.append(1)
                msg_int32.data.append(4)
                self.state_terminal_pub.publish(msg_int32)

                self.is_arrived = True
        else:
            self.is_arrived = False            
        pass

    def sendUDPThread(self):
        while not rospy.is_shutdown():
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

            time.sleep(0.1)

    def terminalCallCallback(self, msg):
        self.var_mtx.acquire()
        self.status_auto = 1
        self.terminal_cmd = msg.data
        self.var_mtx.release()

    def uiCallCallback(self, msg):
        if len(msg.data)>1:
            self.var_mtx.acquire()
            self.status_auto = msg.data[0]
            self.terminal_cmd = msg.data[1]
            self.var_mtx.release()

            if self.status_auto == 1:
                msg_int32 = Int32MultiArray()
                msg_int32.data.append(1)
                msg_int32.data.append(2)
                self.state_terminal_pub.publish(msg_int32)
        