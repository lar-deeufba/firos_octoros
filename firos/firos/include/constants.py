# MIT License
#
# Copyright (c) <2015> <Ikergune, Etxetar>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
# FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
# WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os
import json
import netifaces
try:
    # Python 3
    from urllib.request import urlopen
except ImportError:
    # Python 2
    from urllib2 import urlopen

class Constants:
    configured = False
    PATH = None
    DATA = None
    # All Constants with their default value!
    LOGLEVEL = "INFO"
    INTERFACE = "public"

    MAP_SERVER_ADRESS = None
    MAP_SERVER_PORT = 10100
    ROSBRIDGE_PORT = 9090
    CONTEXT_TYPE = "ROBOT"   
    PUB_FREQUENCY = 0               # In Milliseconds

    ROS_NODE_NAME = "firos"
    ROS_SUB_QUEUE_SIZE = 10 

    @classmethod
    def setConfiguration(cls, path):
        try:
            data = json.load(open(path + "/config.json"))
            return data[data["environment"]]
        except:
            return {}

    @classmethod
    def init(cls, path):
        if not cls.configured:
            cls.configured = True
            cls.PATH = path

            configData = cls.setConfiguration(path)
            cls.DATA = configData

            if "interface" in configData:
                cls.INTERFACE = configData["interface"]
            
            if "log_level" in configData:
                cls.LOGLEVEL = configData["log_level"]

            if "server" in configData and "port" in configData["server"]:
               cls. MAP_SERVER_PORT = configData["server"]["port"]

            if "node_name" in configData:
                cls.ROS_NODE_NAME = configData["node_name"]
            
            if "ros_subscriber_queue" in configData:
                cls.ROS_SUB_QUEUE_SIZE = int(configData["ros_subscriber_queue"])

            if "context_type" in configData:
                cls.CONTEXT_TYPE = configData["context_type"]


            if cls.INTERFACE == "public":
                cls.MAP_SERVER_ADRESS = urlopen('http://ip.42.pl/raw').read()
            else:
                netifaces.ifaddresses(cls.INTERFACE)
                cls.MAP_SERVER_ADRESS = netifaces.ifaddresses(cls.INTERFACE)[2][0]['addr']

            if "rosbridge_port" in configData:
                cls.ROSBRIDGE_PORT = int(configData["rosbridge_port"])

            if "pub_frequency" in configData:
                cls.PUB_FREQUENCY = int(configData["pub_frequency"])