# CubeSat Educational Satellite Platform (Softare Library)
# Yu Hin Hau (Billy)
# billwaahau@gmail.com
# November 2023

import board
import time

import busio
import asyncio
import digitalio
import analogio
import pwmio
import terminalio
import displayio

from adafruit_motor import servo

import adafruit_displayio_ssd1306
from adafruit_display_text import label

import wifi
import ipaddress
import socketpool
from adafruit_httpserver import (
    Server,
    REQUEST_HANDLED_RESPONSE_SENT,
    Request,
    Response,
)

from adafruit_httpserver.headers import Headers
import mdns

from ulab import numpy as np

import adafruit_hcsr04
import adafruit_bmp280


class Satellite:

    def __init__(self):

        # Start Time
        self.t0 = time.time()

        # LED
        self.pin_led = board.IO15
        self.led = digitalio.DigitalInOut(self.pin_led)
        self.led.direction = digitalio.Direction.OUTPUT
        self.led.value = True

        # I2C Interface
        self.pin_scl = board.IO35
        self.pin_sda = board.IO33

        # Deployable Solar Array Servos
        self.pin_servo1 = board.IO37
        self.pin_servo2 = board.IO39
        self.servo1 = None
        self.servo2 = None

        # Sensors
        self.pin_photoresistor = board.IO16
        self.brightnessSensor = None
        self.brightness = -1
        self.brightnessList = []

        # BMP280
        self.tempSensor = None
        self.temperature = -1
        self.pressure = -1

        # Ultrasonic Range Sensor
        self.pin_trig = board.IO3
        self.pin_echo = board.IO5
        self.distance = -1
        self.sonar = None

        # Reset Button
        self.pin_reset = board.IO6
        self.resetButton = None
        self.resetSetup()

        # Buzzer
        self.pin_buzzer = board.IO7
        self.beaconSetup() # To prevent buzzing in the beginning


        # I2C
        self.I2C = None

        # WebServer
        self.webServer = None

        # Sleep a Bit to Avoid Power Spike
        time.sleep(1)

        # Try Setup Display
        self.setupDisplay()

    ########## I2C #######################################

    def setupI2C(self):
        if not self.I2C:
            # Create I2C Bus
            try:
                displayio.release_displays()
                self.I2C = busio.I2C(self.pin_scl, self.pin_sda)
            except:
                pass


    ############ HTTP SERVER #############################
    def startRadio(self, ssid="CubeSat", password="password"):

        ipv4 =  ipaddress.IPv4Address("192.168.1.1")
        netmask =  ipaddress.IPv4Address("255.255.255.0")
        gateway =  ipaddress.IPv4Address("192.168.1.0")
        wifi.radio.set_ipv4_address_ap(ipv4=ipv4,netmask=netmask,gateway=gateway)

        wifi.radio.start_ap(ssid=ssid, password=password)

        print("Access point created with SSID: {}, password: {}".format(ssid, password))
        print("IP address: ", wifi.radio.ipv4_address_ap)


        mdns_server = mdns.Server(wifi.radio)
        mdns_server.hostname = "cubesat"
        mdns_server.advertise_service(service_type="_http", protocol="_tcp", port=80)

        self.displayText(f"{ssid} / {password}\n{wifi.radio.ipv4_address_ap}")

        self.pool = socketpool.SocketPool(wifi.radio)
        self.server = Server(self.pool, "/static", debug=True)

        @self.server.route("/")
        def base(request: Request):
            """
            Serve the default index.html file.
            """
            with open("static\index.html") as f:
                html = f.read()
                html = html.replace("$time$", str(time.time()-self.t0))
                html = html.replace("$brightness$", str(self.brightness))
                html = html.replace("$pressure$", str(self.pressure))
                html = html.replace("$temperature$", str(self.temperature))
                html = html.replace("$range$", str(self.distance))

                return Response(request, html, headers=Headers("Expires: Mon, 15 Jan 2024 00:00:00 GMT") , content_type="text/html")

        self.server.start(str(wifi.radio.ipv4_address_ap))



    def downlink(self, temperature, pressure, brightness, distance):

        self.resetLoop()
        self.brightness = brightness
        self.pressure = pressure
        self.temperature = temperature
        self.distance = distance
        pool_result = self.server.poll() # Process Requests


    ############ OLED Display ############################

    def setupDisplay(self):
        self.display = None
        self.setupI2C()

        if self.I2C:
            try:
                display_bus = displayio.I2CDisplay(self.I2C, device_address=0x3C)
                self.display = adafruit_displayio_ssd1306.SSD1306(display_bus, width=128, height=32)
            except:
                self.display = None


    def displayText(self, text):
        if self.display:
            splash = displayio.Group()
            self.display.root_group = splash
            text_area = label.Label(terminalio.FONT, text=text, color=0xFFFF00, x=2, y=5)
            splash.append(text_area)
            self.display.show(splash)

    # Buzzer
    def beaconSetup(self):
        self.buzzer = digitalio.DigitalInOut(self.pin_buzzer)
        self.buzzer.direction = digitalio.Direction.OUTPUT
        self.buzzer.value = True
        self.led.value = False

        self.beaconState = False
        self.beaconDuration = 1

    def beaconFrequency(self, frequency):
        self.beaconDuration = 1/frequency

    def beaconOn(self):
        self.beaconState = True
        self.beaconT0 = time.time()


    def beaconOff(self):
        self.beaconState = False


    def beaconLoop(self):
        if (self.beaconState):
            if (time.time() - self.beaconT0 >= self.beaconDuration):
                self.beaconT0 = time.time()
                self.beaconBuzz()

    def beaconBuzz(self):
        print('Buzz')
        self.buzzer.value = False
        self.led.value = True
        time.sleep(0.1)
        self.buzzer.value = True
        self.led.value = False

    def beaconAlert(self):
        for i in range(100):
            print('Alert')
            self.buzzer.value = False
            self.led.value = True

            if not self.resetButton.value:
                return True

            time.sleep(0.05)
            self.buzzer.value = True
            self.led.value = False

            if not self.resetButton.value:
                return True

            time.sleep(0.05)

        return False



    ########## SERVO ###################

    def servoSetup(self):
        self.pwm1 = pwmio.PWMOut(self.pin_servo1, duty_cycle=2 ** 15, frequency=50)
        self.servo1 = servo.Servo(self.pwm1)
        self.servo1.angle = 90
        time.sleep(1.5)

        self.pwm2 = pwmio.PWMOut(self.pin_servo2, duty_cycle=2 ** 15, frequency=50)
        self.servo2 = servo.Servo(self.pwm2)
        self.servo2.angle = 90
        time.sleep(1.5)

    def servoSetup(self, id):
        if id == 1:
            self.pwm1 = pwmio.PWMOut(self.pin_servo1, duty_cycle=2 ** 15, frequency=50)
            self.servo1 = servo.Servo(self.pwm1)

        elif id == 2:
            self.pwm2 = pwmio.PWMOut(self.pin_servo2, duty_cycle=2 ** 15, frequency=50)
            self.servo2 = servo.Servo(self.pwm2)


    def servoAngle(self, angle):
        if self.servo1:
            self.servo1.angle = angle
            time.sleep(1.5)

        if self.servo2:
            self.servo2.angle = angle
            time.sleep(1.5)

    def servoAngle(self, angle, id):
        if id == 1:
            if not self.servo1:
                self.servoSetup(1)
            self.servo1.angle = angle

        elif id == 2:
            if not self.servo2:
                self.servoSetup(2)
            self.servo2.angle = angle

    def deploySolarArray(self, countdown=15, timeAlert = 5):

        t0 = time.monotonic()
        durationON = 0.1
        durationOFF = 0.05
        durationToggle = durationON
        toggle = True
        abort = False

        last = countdown

        while countdown > 0 and not abort:
#             print(countdown)
            dt = time.monotonic() - t0
            t0 = time.monotonic()
            countdown -= dt
            durationToggle -= dt

            if int(countdown) != last:
                self.buzzer.value = False
                self.led.value = True
                self.displayText(f"SOLAR ARRAY\nDeployment In... {round(countdown)}")
                print(f"SOLAR ARRAY Deployment In... {round(countdown)}")
                last = int(countdown)
                self.buzzer.value = True
                self.led.value = False


            if countdown < timeAlert and durationToggle < 0:
                toggle ^= 1
                self.buzzer.value = not toggle
                self.led.value = toggle

                if toggle:
                    durationToggle = durationON
                else:
                    durationToggle = durationOFF

            if self.resetButton.value == False:
                abort = True
                print("ABORT!")
                break

            time.sleep(0.01)

        if not abort:
            self.led.value = True
            self.buzzer.value = False
            print('Deploying Solar Array')

            for i in [1,2]:
                self.servoSetup(i)
                self.servoAngle(0, i)
                time.sleep(1)
                self.servoDetach(i)

            self.led.value = False
            self.buzzer.value = True
        else:
            print('Deployment Abort')

        self.servoDetach()


    def servoDetach(self, id = 0):

        if (id == 0 or id == 1):
            if self.servo1:
                self.servo1 = None
                self.pwm1.deinit()
                self.pwm1 = None

        if (id == 0 or id == 2):
            if self.servo2:
                self.servo2 = None
                self.pwm2.deinit()
                self.pwm2 = None

    ############## RESET BUTTON #######################

    def resetSetup(self):
        if not self.resetButton:
            self.resetButton = digitalio.DigitalInOut(self.pin_reset)
            self.resetButton.direction = digitalio.Direction.INPUT
            self.resetButton.pull = digitalio.Pull.UP

    def resetLoop(self):
        if not self.resetButton.value:
            print('Locking R&R')
            self.led.value = True

            for i in [1,2]:
                self.servoSetup(i)
                self.servoAngle(90, i)
                time.sleep(1)
                print(f"[LOCKING] RETENTION SERVO {i} locked!\n")

                self.servoDetach(i)


            self.led.value = False

    ############ Brightness Sensor ####################

    def startBrightnessSensor(self):
        self.brightnessSensor = analogio.AnalogIn(self.pin_photoresistor)

    def readBrightness(self):
        if self.brightnessSensor is None:
            print("[ERROR] Cannot read brightness, the Brightness Sensor is currently OFF! You might want to call cubeSat.startBrightnessSensor() in your code!")
#             time.sleep(0.1)
        else:
            b = round((1 - self.brightnessSensor.value/53023) * 100, 1)

            if b > 100:
                b = 100
            elif b < 0:
                b = 0

            self.brightnessList.append(b)

            if len(self.brightnessList) > 5:
                self.brightnessList.remove(self.brightnessList[0])

            return round(np.mean(self.brightnessList), 1)


    ########### BMP280 #####################

    def startTemperatureSensor(self):
        if self.tempSensor is None:
            self.tempSensor = adafruit_bmp280.Adafruit_BMP280_I2C(self.I2C, 0x76)

    def readTemperature(self):
        if not self.tempSensor is None:
            return round(self.tempSensor.temperature,1)
        else:
            print("[ERROR] Cannot read temperature, the Temperature Sensor is currently OFF! You might want to call cubeSat.startTemperatureSensor() in your code!")

    def startPressureSensor(self):
        if self.tempSensor is None:
            self.startTemperatureSensor()

    def readPressure(self):
        if not self.tempSensor is None:
            return self.tempSensor.pressure
        else:
            print("[ERROR] Cannot read pressure, the Pressure Sensor is currently OFF! You might want to call cubeSat.startPressureSensor() in your code!")



    ####### HC-SR04 ##############################

    def startRangeSensor(self):
        if self.sonar is None:
            self.sonar = adafruit_hcsr04.HCSR04(self.pin_trig, self.pin_echo)

    def readRange(self):
        if not self.sonar is None:
            try:
                return self.sonar.distance
            except:
                print("[ERROR] Problem reading from the Range Sensor... try unplug and plugging back in. Yeah, this sensor don't like soft refresh. It will probably, hopefully work when the satellite is on battery.")
                return -1
        else:
            print("[ERROR] Cannot read distance, the Range Sensor is currently OFF! You might want to call cubeSat.startRangeSensor() in your code!")


class SatelliteTest:

    def __init__(self):
        print("\n\n== CubeSat Bench Test ==")


    def deployableSolarArrayTest(self):
        print("\n-- Deployable Solar Array Calibration & Testing --\n")
        print("[CALIBRATION] For your safety, please manually deploy the Deployable Solar Array...")
        print("[CALIBRATION] Press the RESET BUTTON when the solar arrays are released!\n\n")

        cubeSat = Satellite()

        while cubeSat.resetButton.value:
            pass

        for i in [1,2]:
            cubeSat.servoSetup(i)
            cubeSat.servoAngle(0, i)
            time.sleep(1)

            print(f"[CALIBRATION] RETENTION SERVO {i} is set to 0 deg, adjust servo horn so that it is horizontal!")
            print("[CALIBRATION] Press the RESET BUTTON when you are done!\n\n")


            while cubeSat.resetButton.value:
                pass

            cubeSat.servoDetach(i)


        print("[LOCKING] Now, we will put the CubeSat in the STOWED position to prepare for a deployment test.\n")


        for i in [1,2]:

            print(f"[LOCKING] Hold Down DEPLOYABLE SOLAR ARRAY {i}.")
            print(f"[LOCKING] Press the RESET BUTTON when you are ready to lock RETENTION SERVO {i}!")
            time.sleep(1)

            while cubeSat.resetButton.value:
                pass

            cubeSat.servoSetup(i)
            cubeSat.servoAngle(90, i)
            time.sleep(1)
            print(f"[LOCKING] RETENTION SERVO {i} locked!\n")

            cubeSat.servoDetach(i)


        print(f"[LOCKING] CubeSat Locking Sequence COMPLETED!\n\n")

        print("[DEPLOYMENT] The CubeSat should hopefully be in the STOWED position, we will now do a deployment test!")
        print("[DEPLOYMENT] MAKE SURE THE AREA SURROUNDING THE CUBESAT IS CLEAR!")
        print("[DEPLOYMENT] Press the RESET BUTTON when you are Ready!\n\n")

        time.sleep(1)

        while cubeSat.resetButton.value:
            pass

        print("[DEPLOYMENT] Executing Deployment Sequence... press RESET BUTTON to abort!\n\n")
        time.sleep(2)

        cubeSat.deploySolarArray(countdown=15, timeAlert = 5)

        print("\n\n[DEPLOYMENT] Executing Deployment Sequence COMPLETED!")

        print("[TEST] END OF TESTING SEQUENCE!")








    def fullTest(self):
        cubeSat = Satellite()
        cubeSat.deploySolarArray(countdown=5, timeAlert=1)

        cubeSat.startRadio(ssid="BillySat", password="password")
        cubeSat.startTemperatureSensor()
        cubeSat.startPressureSensor()
        cubeSat.startBrightnessSensor()
        cubeSat.startRangeSensor()

        while True:
        #     cubeSat.beacon(frequency = 0.5)

            temperature = cubeSat.readTemperature()
            pressure = cubeSat.readPressure()
            brightness = cubeSat.readBrightness()
            distance = cubeSat.readRange()

            cubeSat.downlink(temperature, pressure, brightness, distance)






