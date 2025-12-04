import requests
from enum import Enum
from time import sleep

class RESTAPI():
    def __init__(self, robo_ip):
        self.ROBOT_IP = robo_ip
        self.BASE_URL = f"http://{self.ROBOT_IP}/universal-robots/robot-api"

    def throw_exception(self, e):
        print(f"An error occurred: {e}")

    def throw_exception_and_reset(self, e):
        self.ROBOT_IP = None
        self.BASE_URL = None
        self.throw_exception(e)
    
    def check_set(self):
        if(self.ROBOT_IP is None or self.BASE_URL is None):
            self.throw_exception("ROBOT_IP and BASE_URL not set!")
            return -1, f"{self.ROBOT_IP=} and {self.BASE_URL=} not set!"
        return 0, f"{self.ROBOT_IP=} and {self.BASE_URL=} set!"

    def get_system_time(self):
        set_status, set_msg = self.check_set()
        if(set_status == -1):
            return set_status, set_msg
        url = f"{self.BASE_URL}/system/v1/system-time"
        try:
            response = requests.get(url)
            response.raise_for_status()

            data = response.json()
            print("System Time Info:")
            print(data)
            return 0, str(data)
        except requests.exceptions.RequestException as e:
            self.throw_exception(e)
            return -1, str(e)
    
    def test_connection(self):
        set_status, set_msg = self.check_set()
        if(set_status == -1):
            return set_status, set_msg
        time_status, time_msg = self.get_system_time()
        if(time_status == -1):
            self.throw_exception_and_reset("Unable to test system time!")
            return time_status, time_msg
        return 0, "Connection test successful!"
    def try_put(self, url, headers, payload):
        try:
            """http://128.138.224.247/universal-robots/robot-api/docs#/"""
            """https://requests.readthedocs.io/en/latest/"""
            response = requests.put(url, json=payload, headers=headers)
            response.raise_for_status()
            sleep(0.5)
        except requests.exceptions.RequestException as e:
            self.throw_exception(e)
            return -1, str(e)
        return 0, f"sucessfully put {url=}, {headers=}, {payload=}"
    
    def try_get(self, url):
        try:
            response = requests.get(url)
            response.raise_for_status()
            data = response.json()
            return 0, str(data)
        except requests.exceptions.RequestException as e:
            self.throw_exception(e)
            return -1, str(e)

    def get_program_state(self):
        set_status, set_msg = self.check_set()
        if(set_status == -1):
            return set_status, set_msg
        url = f"{self.BASE_URL}/program/v1/state"
        get_status, get_msg = self.try_get(url)
        return get_status, get_msg


    def check_ready(self):
        set_status, set_msg = self.check_set()
        if set_status == -1:
            return False, set_msg
        state_status, state_msg = self.get_program_state()
        if state_status == -1:
            print("[check_ready] Unable to get program state!")
            return False, state_msg
        
        # TODO might bring back state checking later
        # if msg['state'] != 'STOPPED':
        #     self.throw_exception("[check_ready] The robot is currently running a program!")
        #     return False
        return True, "Robot is ready!"


    def load_program(self, program_name):
        ready_bool, ready_msg = self.check_ready()
        if not ready_bool:
            return -1, ready_msg
        url = f"{self.BASE_URL}/program/v1/load"
        payload = {"programName": program_name}
        headers = {"accept": "application/json", 
                   "Content-Type": "application/json"}
        put_status, put_msg =  self.try_put(url, headers, payload)
        return put_status, put_msg
        
    def play_program(self):
        ready_bool, ready_msg = self.check_ready()
        if not ready_bool:
            return -1, ready_msg
        url = f"{self.BASE_URL}/program/v1/state"
        payload = {"action": "play"}
        headers = {"accept": "application/json", 
                   "Content-Type": "application/json"}
        play_status, play_msg = self.try_put(url, headers, payload)
        return play_status, play_msg

    def MoveUntilContact(self):
        return self.load_program("Contact")
    def Retract(self):
        return self.load_program("Retract")

    def ros2_control(self):
        #OPTIONAL NOT REALLY NEEDED, but its nice if the tablet reads external control
        return self.load_program("ExternalControl")

    def send_state(self, state):
        set_status, set_msg = self.check_set()
        if set_status == -1:
            return set_status, set_msg
        url = f"{self.BASE_URL}/robotstate/v1/state"
        headers = {"accept": "application/json", "Content-Type": "application/json"}
        payload = {"action": state}
        put_status, put_msg = self.try_put(url, headers, payload)
        return put_status, put_msg

    def unlock_robot(self):
        status, msg = self.send_state("BRAKE_RELEASE")
        return status, msg
        
    def power_on_robot(self):
        status, msg = self.send_state("POWER_ON")
        return status, msg

    def power_off_robot(self):
        status, msg = self.send_state("POWER_OFF")
        return status, msg

        
            
