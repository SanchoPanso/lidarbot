import RPi.GPIO as GPIO
import time

class PWMController:
    def __init__(self, pin, frequency=50):
        self.pin = pin
        self.frequency = frequency
        self.pwm = None

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, self.frequency)
        self.pwm.start(0)

    def change_duty_cycle(self, duty_cycle):
        self.pwm.ChangeDutyCycle(duty_cycle)

    def breathe(self, duration=10, step=1, delay=0.01):
        end_time = time.time() + duration
        while time.time() < end_time:
            for duty_cycle in range(0, 101, step) + range(100, -1, -step):
                self.change_duty_cycle(duty_cycle)
                time.sleep(delay)

    def stop(self):
        if self.pwm:
            self.pwm.stop()

    def cleanup(self):
        GPIO.cleanup()

    def __del__(self):
        self.stop()
        self.cleanup()
