import requests
from enum import Enum
from time import sleep

class robotState(Enum):
    OFF = 0
    ACTIVE = 1
class RESTAPI():
    def __init__(self, robo_ip):
        self.ROBOT_IP = robo_ip
        self.BASE_URL = f"http://{self.ROBOT_IP}/universal-robots/robot-api"
        #TODO: activate the robot?
        self.robot_state = robotState.OFF

    def throw_exception(self, e):
        print(f"An error occurred: {e}")

    def throw_exception_and_reset(self, e):
        self.ROBOT_IP = None
        self.BASE_URL = None
        self.throw_exception(e)
    
    def check_set(self):
        if(self.ROBOT_IP == None or self.BASE_URL == None):
            self.throw_exception("ROBOT_IP and BASE_URL not set!")
            return -1
        return 0, None

    def get_system_time(self):
        if(self.check_set() == -1):
            return -1, None
        url = f"{self.BASE_URL}/system/v1/system-time"
        try:
            response = requests.get(url)
            response.raise_for_status()

            data = response.json()
            print("System Time Info:")
            print(data)
            return 0, data
        except requests.exceptions.RequestException as e:
            self.throw_exception(e)
            return -1, None
    
    def test_connection(self):
        if(self.check_set() == -1):
            return -1
        status, _ = self.get_system_time()
        if(status == -1):
            self.throw_exception_and_reset("Unable to test system time!")
            return -1
        return 0
    def try_put(self, url, headers, payload):
        try:
            """http://128.138.224.247/universal-robots/robot-api/docs#/"""
            """https://requests.readthedocs.io/en/latest/"""
            response = requests.put(url, json=payload, headers=headers)
            response.raise_for_status()
            sleep(0.5)
        except requests.exceptions.RequestException as e:
            self.throw_exception(e)
            return -1, None
        return 0, None
    
    def try_get(self, url):
        try:
            response = requests.get(url)
            response.raise_for_status()
            data = response.json()
            return 0, data
        except requests.exceptions.RequestException as e:
            self.throw_exception(e)
            return -1, None

    def get_program_state(self):
        if(self.check_set() == -1):
            return -1, None
        url = f"{self.BASE_URL}/program/v1/state"
        return self.try_get(url)
        
    def CRMove(self, contact):#contact (bool) -> true if move into cotact, false if retract
        if self.check_set() == -1:
            return -1, None
        # ensure the robot is active
        if self.robot_state != robotState.ACTIVE:
            self.throw_exception("The robot is not active!")
            return -1, None
        status, msg = self.get_program_state()
        if msg['state'] != 'STOPPED':
            self.throw_exception("The robot is currently running a program!")
            return -1, None
        
        url = f"{self.BASE_URL}/program/v1/load"
        if contact:
            payload = {"programName": "Contact"}
        else:
            payload = {"programName": "Retract"}
        headers = {"accept": "application/json", 
                   "Content-Type": "application/json"}
        status, msg =  self.try_put(url, headers, payload)
        if status == -1:
            return status, msg
         
        url = f"{self.BASE_URL}/program/v1/state"
        payload = {"action": "play"}

        return self.try_put(url, headers, payload)

    def LRSet(self, release):#release bool, if true release, if false lock
        if self.check_set() == -1:
            return -1, None
        url = f"{self.BASE_URL}/robotstate/v1/state"
        headers = {"accept": "application/json", "Content-Type": "application/json"}
        if release:
            # we want to release the robot brakes
            if self.robot_state == robotState.ACTIVE:
                return 0, None
    
            payload = {"action": "POWER_ON"}
            status, msg = self.try_put(url, headers, payload)
            if status == -1:
                return status, msg
            payload = {"action": "BRAKE_RELEASE"}
            status, msg = self.try_put(url, headers, payload)
            if status == 0:
                self.robot_state = robotState.ACTIVE
            return status, msg
        
        else:
            # we want to lock the robot
            if self.robot_state == robotState.OFF:
                return 0, None
            payload = {"action": "POWER_OFF"}
            status, msg = self.try_put(url, headers, payload)
            if status == 0:
                self.robot_state = robotState.OFF
            return status, msg
