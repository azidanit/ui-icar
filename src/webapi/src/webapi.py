import rospy
from threading import Thread, Lock
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8
import time
import json
from urllib.request import Request, URLError, urlopen
from urllib.parse import urlencode


import os

class WebAPI:
    def __init__(self):
        self.url = "https://riset.its.ac.id/icar/api/"
        self.cmd_post_location = "poslocation"
        self.cmd_get_terminal  = "getCall"
        self.thread_get = None
        self.thread_post = None
        # -7.277766, 112.797430
        self.current_position = {'id':1, 'latitude': -7.277766, 'longitude': 112.797430}
        self.position_lock = Lock()

        self.lat = -7.277766
        self.long = 112.797430
        pass
        # self.openLastFile()

    def start(self):
        self.terminal_pub = rospy.Publisher("/webapi/terminal_call", Int8, queue_size=1)
        self.position_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, callback=self.position_callback)
        self.thread_post = Thread(target=self.run_post_position)
        self.thread_post.start()
        self.thread_get = Thread(target=self.run_get_terminal)
        self.thread_get.start()
        rospy.spin()

    def send_request(self, url, param=None):
        # send_request
        resp = 0
        if param is not None:
            data = urlencode(self.current_position).encode()
            req = Request(url, data=data)
        else:
            req = Request(url)

        try:
            resp = urlopen(req, timeout=2)
        except URLError as e:
            return 0
        except:
            return 0
        print("CODE: ", url, resp.getcode())
        # print(resp.read().decode())
        resp_dec = resp.read().decode()
        return resp_dec

    def run_get_terminal(self):
        terminal_msg = Int8()
        while not rospy.is_shutdown():
            # get go_to_terminal command via HTTP 
            url = self.url + self.cmd_get_terminal
            resp = self.send_request(url=url)
            if resp==0 :
                pass
            else:
                print(resp)
                if(resp!=""):
                    resp_obj = json.loads(resp)
                    terminal_msg.data = int(resp_obj['terminal_id'])
                    print(terminal_msg.data, type(terminal_msg.data))
                    if(terminal_msg.data <= 6):
                        print("KIRIM ", terminal_msg.data, type(terminal_msg.data))
                        self.terminal_pub.publish(terminal_msg)
            time.sleep(2)
    
    def run_post_position(self):
        while not rospy.is_shutdown():
            time.sleep(2)
            url = self.url + self.cmd_post_location
            if self.current_position['latitude'] == 0:
                self.current_position['latitude'] = self.default_lat
                self.current_position['longitude'] = self.default_long
            print('Send position: {}'.format(self.current_position))
            resp = self.send_request(url = url, param=self.current_position)
            print('POST : ', resp)
            self.writeFile()
            
    
    def position_callback(self, msg):
        lat_ = "{:.7f}".format(msg.latitude)
        long_ = "{:.7f}".format(msg.longitude)
        self.position_lock.acquire()
        self.current_position['latitude'] = lat_
        self.current_position['longitude'] = long_
        self.position_lock.release()

    def __del__(self):
        if self.thread_get.is_alive():
            self.thread_get.join()
        if self.thread_post.is_alive():
            self.thread_post.join()



    def openLastFile(self):
        dir = "last_position.txt"
        print("OPENING LAST FILE")
        if not os.path.isfile(dir):
            print("FILE NOT FOUND, making new file")
            self.writeFile()
        else:
            print("Opening file")
            f = open(dir, "r")
            last_lat_long = f.read()
            last_lat_long = last_lat_long.split(',')
            self.default_lat = last_lat_long[0]
            self.default_long = last_lat_long[1]
            self.current_position['latitude'] = last_lat_long[0]
            self.current_position['longitude'] = last_lat_long[1]
        pass

    def writeFile(self):
        dir = "last_position.txt"
        print("writing file")
        f = open(dir, "w")
        str_msg = "{},{}".format(self.current_position['latitude'], self.current_position['longitude'])
        f.write(str_msg)
        f.close()
