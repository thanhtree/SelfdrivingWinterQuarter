import pigpio
import time
import socket

UDP_IP ="192.168.2.12" # IP Address of Raspberry Pi
UDP_PORT_SPEED = 5006 # UDP Port for sending speed; this should be different from other UDP ports that are being used to send data to the Raspberry Pi

sock_speed = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

hall_effect_sensor_pin = 23
pi = pigpio.pi()
pi.set_pull_up_down(hall_effect_sensor_pin, pigpio.PUD_DOWN)
pi.set_mode(hall_effect_sensor_pin, pigpio.INPUT)
delta_time = -1 # Initialize time it took to detect magnet to negative one
count = 0 # this is to check if we are at the first iteration of the while loop, in which case, there is no previous_hall_effect_sensor_state, so we initialize the current_hall_effect_sensor_state
delta_time_count = 0 # this is to check the number of times we receive a change in the delta_time; every time the delta_time variable changes, that means the magnet has passed by again; for whatever reason, when the magnet passes by the first time, delta_time is an insanely high number that does not represent the true delta_time, so we will ignore the first delta_time we receive 
actual_speed = -999901.0 # Initialize actual speed of car (inches/second) to an impossible value so we know if we are still reading the initialized speed
while True:
    if (count == 0):
        current_hall_effect_sensor_state = pi.read(hall_effect_sensor_pin)
        count += 1
        t1 = time.time()
    else:
        current_hall_effect_sensor_state = pi.read(hall_effect_sensor_pin)
        if (current_hall_effect_sensor_state == 0 and previous_hall_effect_sensor_state == 1):
            t2 = time.time()
            delta_time = t2-t1
            actual_speed = 2.0/delta_time
            t1 = t2
            #time.sleep(0.05)
    previous_hall_effect_sensor_state = current_hall_effect_sensor_state
    actual_speed_string = str(actual_speed)
    sock_speed.sendto(bytes(actual_speed_string, "utf-8"), (UDP_IP, UDP_PORT_SPEED))
    print("delta_time: ", delta_time)
    print("actual_speed: ", actual_speed)
    print()
    #time.sleep(0.005)
    
