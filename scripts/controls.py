class MockGPIO:
    OUT = "OUT"
    IN = "IN"
    BCM = "BCM"

    class PWM:
        def __init__(self, pin, frequency):
            self.pin = pin
            self.frequency = frequency
            print(f"Mock PWM created for pin {pin} with frequency {frequency}")

        def start(self, duty_cycle):
            print(f"Mock PWM on pin {self.pin} started with duty cycle {duty_cycle}%")

        def ChangeDutyCycle(self, duty_cycle):
            print(f"Mock PWM on pin {self.pin} changed duty cycle to {duty_cycle}%")

        def stop(self):
            print(f"Mock PWM on pin {self.pin} stopped")


try:
    import RPi.GPIO as GPIO

    ON_RASPBERRY_PI = True
except (ImportError, RuntimeError):
    GPIO = MockGPIO
    ON_RASPBERRY_PI = False


class GPIOHandler:
    def __init__(self, mock_mode=False):
        self.mock_mode = mock_mode or not ON_RASPBERRY_PI
        if self.mock_mode:
            print("Mock mode activated. No real GPIO will be used.")
        else:
            GPIO.setmode(GPIO.BCM)

    def setup_pin(self, pin, mode):
        if not self.mock_mode:
            GPIO.setup(pin, mode)
        else:
            print(f"Mock Setup: Pin {pin}, Mode {mode}")

    def pwm(self, pin, frequency):
        if self.mock_mode:
            return MockGPIO.PWM(pin, frequency)
        else:
            return GPIO.PWM(pin, frequency)

    def cleanup(self):
        if not self.mock_mode:
            GPIO.cleanup()
        else:
            print("Mock Cleanup")


class MotorHandler:
    def __init__(self, left_pins, right_pins, gpio_handler):
        self.gpio = gpio_handler
        self.pwm_controls = {}
        self.left_pins = left_pins
        self.right_pins = right_pins

        for pin in left_pins + right_pins:
            self.gpio.setup_pin(pin, GPIO.OUT)
            pwm = self.gpio.pwm(pin, 100)  # 100 Гц
            pwm.start(0)
            self.pwm_controls[pin] = pwm

    def _set_wheels_speed(self, left_speed, right_speed):
        # Здесь нужно адаптировать управление скоростью с ШИМ
        for pin, pwm in self.pwm_controls.items():
            if pin in self.left_pins:
                pwm.ChangeDutyCycle(left_speed)
            elif pin in self.right_pins:
                pwm.ChangeDutyCycle(right_speed)

    def move_forward(self, speed):
        self._set_wheels_speed(speed, speed)

    def move_backward(self, speed):
        self._set_wheels_speed(-speed, -speed)

    def turn_right(self, speed):
        self._set_wheels_speed(speed, -speed)

    def turn_left(self, speed):
        self._set_wheels_speed(-speed, speed)

    def stop(self):
        self._set_wheels_speed(0, 0)

    def __del__(self):
        for pwm in self.pwm_controls.values():
            pwm.stop()
        self.gpio.cleanup()
        
        
if __name__ == "__main__":
    # Пример использования
    gpio_handler = GPIOHandler(mock_mode=True)  # Или mock_mode=True для отладки
    motor_handler = MotorHandler([17], [27], gpio_handler)
