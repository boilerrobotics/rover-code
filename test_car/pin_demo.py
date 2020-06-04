import Jetson.GPIO as GPIO #allows us to interface with pins
import time

output_pin = 18 #

def main():
	GPIO.setmode(GPIO.BCM) #maps "output_pin" to something the library understands
	GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH) #set pin "18" for output and make it high
	print("Press CTRL+C to gtfo")
	
	#i'll be honest, the syntax below is kinda fucky but i guess this is how they do python hardware programming....rick would never
	pin_state = GPIO.HIGH
	try:
		while True:
			time.sleep(1)
			print("sending {} to pin {}".format(pin_state, output_pin)) #just terminal printing for our sanity
			GPIO.output(output_pin, pin_state) #actually changes pin value
			pin_state ^= GPIO.HIGH #alternates
	finally:
		GPIO.cleanup() #resets pins to default state

if __name__ == '__main__':
	main()
