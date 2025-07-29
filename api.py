from esp32bot import ESP32bot
import time

robot = ESP32bot("192.168.195.144")
#robot.motor_a_forward()
#obot.motor_b_forward()
robot.motor_a_stop()
robot.motor_b_stop()
while True:
    robot.icm_read()
    robot.bno_read()
    #robot.ir_read()
    #time.sleep(5)
    #robot.close() 
robot.close()   