# feeder for mouse, 2025-07-05 by malo

from machine import Pin, PWM
import time
from umqtt.simple import MQTTClient
import dht
import ubinascii
import machine
import network

# Define the GPIO pin connected to the servo signal line
# For ESP8266, common pins include 0, 2, 4, 5, 12, 13, 14, 15, 16.
'''
D0 = 16
D1 = 5  #PWM
D2 = 4  #PWM
D3 = 0  #PWM
D4 = 2  #PWM, #Led on board
D5 = 14 #PWM
D6 = 12 #PWM
D7 = 13 #PWM
D8 = 15 #PWM
'''
SERVO_PIN = 4

# Initialize PWM on the chosen pin with a frequency of 50Hz
# SG90 servos typically operate at 50Hz
servo_pwm = PWM(Pin(SERVO_PIN), freq=50)

# Define duty cycle values for 0, 90, and 180 degrees.
# These values can vary slightly between individual SG90 servos.
# The ESP8266 PWM duty cycle ranges from 0 to 1023.
# These values are based on common observations for SG90 and a 50Hz frequency:
# 0 degrees: ~26 (equivalent to ~0.5ms pulse width)
# 90 degrees: ~74 (equivalent to ~1.5ms pulse width)
# 180 degrees: ~123 (equivalent to ~2.5ms pulse width)
# You might need to fine-tune these values for your specific servo.
DUTY_0_DEGREES = 26
DUTY_90_DEGREES = 74
DUTY_180_DEGREES = 123

def set_angle(angle):
    """
    Sets the servo to a specific angle (0-180 degrees).
    Maps the angle to the corresponding PWM duty cycle.
    """
    if angle < 0:
        angle = 0
    elif angle > 180:
        angle = 180

    # Linear mapping from angle to duty cycle
    # duty = (angle / 180) * (DUTY_180_DEGREES - DUTY_0_DEGREES) + DUTY_0_DEGREES
    # Using specific values to set 0, 90, 180 directly
    if angle == 0:
        duty = DUTY_0_DEGREES
    elif angle == 90:
        duty = DUTY_90_DEGREES
    elif angle == 180:
        duty = DUTY_180_DEGREES
    else:
        # Interpolate for other angles
        if angle < 90:
            duty = int(DUTY_0_DEGREES + (angle / 90) * (DUTY_90_DEGREES - DUTY_0_DEGREES))
        else:
            duty = int(DUTY_90_DEGREES + ((angle - 90) / 90) * (DUTY_180_DEGREES - DUTY_90_DEGREES))

    servo_pwm.duty(duty)
    print(f"Setting servo to {angle} degrees (Duty: {duty})")

led = Pin(2, Pin.OUT, value=1)
my_new_msg = None
TOPIC_BASE = '/malo/esp'

#Control Function
def led_onoff(onoff):
    """ control led ON or OFF
        parameter:
        onoff
            0-->ON, 1-->OFF (acturely, led ON when level=0)
    """
    global led
    
    if(onoff==1):
        led.value(0)
    elif(onoff==-1):
        led.value(not led.value())
    else:
        led.value(1)

def sub_cb(topic, msg):
    global my_new_msg
    global TOPIC_BASE
    topic_light = TOPIC_BASE + "/light"
    topic_mode = TOPIC_BASE + "/mode"

    topic = topic.decode('utf-8')
    msg = msg.decode('utf-8')
    my_new_msg = '[' + topic + '] ' + msg
    print(my_new_msg)

    if topic == topic_light:
        led_onoff(0 if msg == "0" else 1)

    elif topic == topic_mode:
        try:
            n = int(msg)
            set_angle(n*45)
        except Exception as e:
            print("Invalid n:", msg, "Error:", str(e))


print("Servo control program started.")



def main():
    global my_new_msg
    global TOPIC_BASE
    
    mq_fail_count = 0

    led_onoff(1)

    #- check ap config file
    AP_SSID = 'malo-home'
    AP_PWD = '85074208'
    
    # Default MQTT server to connect to
    server = "iot.eclipse.org"
    CLIENT_ID = ubinascii.hexlify(machine.unique_id()).decode('utf-8')
    topic_light = TOPIC_BASE+"/light"
    topic_msg = TOPIC_BASE+'/msg'
    topic_mode = TOPIC_BASE+"/mode"
    

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(AP_SSID, AP_PWD)
    print('connecting to AP')
    while(not wlan.isconnected()):
        print(wlan.ifconfig())
        time.sleep(0.1)
        led_onoff(-1)
    print('connected!  --> ', wlan.ifconfig())

    c = MQTTClient(CLIENT_ID, server)
    # Subscribed messages will be delivered to this callback
    c.set_callback(sub_cb)
    c.connect()
    c.subscribe(topic_light)
    c.subscribe(topic_mode)
    print("Connected to %s, subscribed to %s topic" % (server, topic_light))

    # wifi ready, blink led
    for i in range(3):
        led_onoff(1)
        time.sleep(1)
        led_onoff(0)
        time.sleep(1)
    print('I am ready!, ID='+str(CLIENT_ID))
    c.publish(topic_msg, 'I am ready!, ID='+str(CLIENT_ID))

    try:
        while 1:
            if(not wlan.isconnected()):
                # not do any mq operation
                time.sleep(0.1)
                led_onoff(-1)                
                continue
            
            try:
                #c.wait_msg()
                c.check_msg()
                if my_new_msg:
                    c.publish(topic_msg, my_new_msg)
                    my_new_msg = None
                    
            except Exception as e:
                print('wlan:', wlan.isconnected())
                print('ex: ', str(e))
                mq_fail_count+=1
                time.sleep(1)
                
            try:
                if mq_fail_count>5:
                    mq_fail_count=0
                    c = MQTTClient(CLIENT_ID, server)
                    # Subscribed messages will be delivered to this callback
                    c.set_callback(sub_cb)
                    c.connect()
                    c.subscribe(topic_light)
                    c.subscribe(topic_mode)
                    print("Connected to %s, subscribed to %s topic" % (server, topic_light))
            except Exception as e:
                print('wlan:', wlan.isconnected())
                print('ex: ', str(e))
                    

            time.sleep(0.001)
                        
    finally:
        c.disconnect()


if __name__ == '__main__':
    main()