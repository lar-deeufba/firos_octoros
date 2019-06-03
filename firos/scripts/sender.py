#!/usr/bin/env python

import sys
import os
import time
import csv
import requests
import json
import rospy
from octo_ros.msg import PrinterState

import requests

head = {"Content-Type": "application/json"}
class OrionClient:
    def __init__(self, host, port=1026):
        self.host=host
        self.port=port

    def createEntity(self, data):
        jsonData = json.dumps(data)

        url = 'http://'+self.host+':'+str(self.port)+'/v2/entities'
        response = requests.post(url, data=jsonData, headers=head)
        print(response)
        try:
            print(response.json())
        except Exception:
            pass


    def queryEntity(self, entity):
        url = 'http://'+self.host+':'+str(self.port)+'/v2/entities/'+ entity
        response = requests.get(url)
        print(response.text)
        #'http://'+host+':1026/v2/entities/Room1?options=values&attrs=temperature,pressure'

    def updateEntity(self,entity,data, force=False):
        jsonData = json.dumps(data)
        if force:
            url = 'http://'+self.host+':'+str(self.port)+'/v2/entities/'+ entity+'/attrs?options=forcedUpdate'
        else:
            url = 'http://'+self.host+':'+str(self.port)+'/v2/entities/'+ entity+'/attrs'
        response = requests.post(url, data=jsonData, headers=head)
        print(response)
        try:
            print(response.json())
        except Exception:
            pass


    def subscription(self,entityType):
        data = {
          "orionUrl": 'http://orion:1026/v2',
          "quantumleapUrl":'http://quantumleap:8668/v2',
          "entityType": entityType
        }

        url = 'http://'+self.host+':8668/v2/subscribe'
        response = requests.post(url,params=data)
        print(response)
        try:
            print(response.json())
        except json.decoder.JSONDecodeError:
            pass

    def getEntities(self, entityType=None):
        if entityType is None:
            url = 'http://'+self.host+':'+str(self.port)+'/v2/entities'
        else:
            url = 'http://'+self.host+':'+str(self.port)+'/v2/entities?type='+entityType
        response = requests.get(url)
        print(response.text)

def callback(data):
    msg = dict()
    msg["date_time"]={"value": data.date_time,"type": "Text"}
    msg["temp_tool1_actual"]={"value": float(data.temp_tool1_actual),"type": "Number"}
    msg["temp_tool2_actual"]={"value": float(data.temp_tool2_actual),"type": "Number"}
    msg["temp_bed_actual"]={"value": float(data.temp_bed_actual),"type": "Number"}
    msg["file_name"]={"value": data.file_name,"type": "Text"}
    msg["printer3d_state"]={"value": data.printer3d_state,"type": "Text"}
    msg["progress"]={"value": float(data.progress),"type": "Number"}
    msg["time_left"]={"value": int(data.time_left),"type": "Number"}
    msg["time_elapsed"]={"value": int(data.time_elapsed),"type": "Number"}
    msg["temp_tool1_goal"]={"value": float(data.temp_tool1_goal),"type": "Number"}
    msg["temp_tool2_goal"]={"value": float(data.temp_tool2_goal),"type": "Number"}
    msg["temp_bed_goal"]={"value": float(data.temp_bed_goal),"type": "Number"}
    print(msg)

    client.updateEntity(entity,msg)

def listener():
    rospy.init_node('tsk_listener', anonymous=True)
    rospy.Subscriber("printer3d", PrinterState, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        current_path = os.path.dirname(os.path.abspath(__file__))
        json_path = current_path.replace("scripts", "config/config.json")
        data = json.load(open(json_path))
        configData = data[data["environment"]]
        host = configData["contextbroker"]["address"]
        port = configData["contextbroker"]["port"]
        print(host, port)
    except:
        print("Config file not found...", json_path)
        sys.exit()

    head = {"Content-Type": "application/json"}
    entity= '3dprinter1'
    print("Connection to the server ", host, " on the port ", port)
    client = OrionClient(host, port)

    listener()
