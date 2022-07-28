import rospy
from threading import Thread, Lock
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int8
import time
import json
from urllib.request import Request, URLError, urlopen
from urllib.parse import urlencode


class WebAPI:
    def __init__(self):
        self.url = "https://riset.its.ac.id/icar/api/"
        self.cmd_post_location = "poslocation"
        self.cmd_get_terminal  = "getCall"
        self.thread_get = None
        self.thread_post = None
        self.current_position = {'id':1, 'latitude': 0, 'longitude': 0}
        self.position_lock = Lock()

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
                    if(terminal_msg.data <= 3):
                        print("KIRIM ", terminal_msg.data, type(terminal_msg.data))
                        self.terminal_pub.publish(terminal_msg)
            time.sleep(2)
    
    def run_post_position(self):
        while not rospy.is_shutdown():
            url = self.url + self.cmd_post_location
            print('Send position: {}'.format(self.current_position))
            resp = self.send_request(url = url, param=self.current_position)
            print('POST : ', resp)
            time.sleep(2)
    
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