from core import RESTAPI
import time

if __name__ == '__main__':
    api = RESTAPI("128.138.224.247")
    status = api.test_connection()
    print(f"Test Connection Status: {status}")

    unlock_out = api.unlock_robot()
    print(f"Unlock Robot Output: {unlock_out}")
    time.sleep(1)


    Retract_out = api.Retract()
    print(f"Retract Output: {Retract_out}")
    time.sleep(1)


    # api.cheat_state_active()
    movetocontact = api.MoveUntilContact()
    print(f"Move Until Contact Output: {movetocontact}")
    time.sleep(1)

    Retract_out = api.Retract()
    print(f"Retract Output: {Retract_out}")
    time.sleep(1)


    lock_out = api.lock_robot()
    print(f"Lock Robot Output: {lock_out}")
    time.sleep(1)


