#Libraries
import RPi.GPIO as GPIO
import socket
import struct
import sys
import time
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('192.168.1.119', 10000)
sock.bind(server_address)
sock.listen(1)
# values = ('1, b'ab', 2.7')
packer = struct.Struct('f')

 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    dist = (StopTime - StartTime) * 34300 / 2
    # print int(dist) , dist
    return dist


if __name__ == '__main__':
    try:
        connection, client_address = sock.accept()
        while True:
            packed_data = packer.pack(distance())
            connection.sendall(packed_data)
	    time.sleep(0.5)

 
        # Reset by pressing CTRL + C
    except:
        GPIO.cleanup()

    finally:
        print('closing socket')
        sock.close()
