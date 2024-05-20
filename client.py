import socketio #import socketio methods
import time #import time methods
import json #import json methods
import os
import digitalio #import gpio methods
import board #import SPI
from RPLCD.i2c import CharLCD
import adafruit_max31865 #import MAX31865 methods
import threading
import schedule
import gpiozero
from  gpiozero import LED, Button, CPUTemperature, RotaryEncoder

HOST = '192.168.0.100' #set hostname
PORT = 3000 #set port number
DEVICE_NAME = 'DEV001' #set device name
CONNECTION_QUERY = f'http://{str(HOST)}:{str(PORT)}' #connection string

# Starting conditions
global newTemp, emgFlag, maxLevel, levelStr, startStatus, stopStatus, menuFlag, menuOptions, bounceTime
bounceTime = 0.1
levelStr = 'OK'
newTemp = 10
menuOptions = 0
emgFlag = False
menuFlag = False
startStatus = False
stopStatus = True
mysocket = socketio.Client()
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, 
              cols=20, rows=4, dotsize=10, auto_linebreaks=False,
              backlight_enabled=True)
lcd.cursor_mode = 'hide'
spi = board.SPI()

# set led output pin 1 On/Off indicator
led1 = LED(5)
# set led output pin 2
led2 = LED(25)
# set led output pin 3 Level sensor indicator
led3 = LED(24)
# set led output pin 4
led4 = LED(23)
# set led output pin 5 Emergency/Error Indicator
led5 = LED(18)

# relay module is operating in reverse mode 0 --> true / 1 --> false
output1 = gpiozero.DigitalOutputDevice(13, initial_value=True)
# relay module is operating in reverse mode 0 --> true / 1 --> false
output2 = gpiozero.DigitalOutputDevice(19, initial_value=True)
# ----------------------------------------------------------
# START BUTTON
def startBtnPressed():
    global startStatus, dataFileName, stopStatus
    if stopButton.is_pressed is True:
        return 0
    else:
        if emgFlag is False:
            if led1.is_active is False:
                startStatus = True
                stopStatus = False
                led1.on()
                checkTemp()
                # output1.off() # relay module is operating in reverse mode 0 --> true / 1 --> false
                output2.off() # relay module is operating in reverse mode 0 --> true / 1 --> false

                # All data are saved to a json file locally. The name of the file is genareted from date and time
                # print(time.strftime('%Y%j%H%M%S'))
                theData = {
                    'action': 'Start Button',
                    'EMG': emgFlag,
                    'timeStamp': now,
                    'sensorA': [tempA, resA],
                    'sensorB': [tempB, resB],
                    'status': startStatus,
                    'cpuTemp': cpu,
                    'userTemp': newTemp,
                    'tankLevel': maxLevel
                }
                theData = json.dumps(theData)
                dataFileName = f'{time.strftime("%Y%j%H%M%S")}'
                print(startStatus)
                print(f'Data to be saved ----> {theData}')
                f = open(f'{dataFileName}.json', 'w')
                f.write('{\n\t"Op_details" : [\n\t\t' + theData + ',' + '\n')
                f.close

    print(f'____________________Start Button pressed! {led1.is_active}')

startButton = gpiozero.Button(27, bounce_time = bounceTime)
startButton.when_pressed = startBtnPressed

# ----------------------------------------------------------
# STOP BUTTON
def stopBtnPressed():
    global startStatus, stopStatus, emgFlag
    if stopStatus is False:
        startStatus = False
        stopStatus = True
        led1.off()
        output1.on() # relay module is operating in reverse mode 0 --> true / 1 --> false
        output2.on() # relay module is operating in reverse mode 0 --> true / 1 --> false
        # TO-DO add other activities to stop
        theData = {
            'action': 'Stop Button',
            'EMG': emgFlag,
            'timeStamp': now,
            'sensorA': [tempA, resA],
            'sensorB': [tempB, resB],
            'status': startStatus,
            'cpuTemp': cpu,
            'userTemp': newTemp,
            'tankLevel': maxLevel
        }
        theData = json.dumps(theData)
        print(startStatus)
        print(f'Data to be saved ----> {theData}')

        f = open(f'{dataFileName}.json', 'a')
        f.write('\t\t' + theData + '\n\t]\n}')
        f.close
    print(f'____________________Stop Button pressed! {led1.is_active}')

def resetDevice():
    global newTemp, emgFlag, menuFlag
    if emgFlag is True: # check if emergency button is pressed, to perform error reset
        newTemp = 10
        emgFlag = False
        menuFlag = False
        led5.blink(on_time=0.5, off_time=0.5, n = 4)

stopButton = gpiozero.Button(16, bounce_time = bounceTime)
stopButton.when_pressed = stopBtnPressed
stopButton.hold_time = 2 # Hold stop button for 2 seconds to perform error reset
stopButton.when_held = resetDevice
# ----------------------------------------------------------
# rotary encoder RIGHT
def rotaryRight():
    global newTemp, menuOptions
    if menuFlag is False:
        newTemp = newTemp + 1
        print(f'__________________________Rotary Right! New Temp : {newTemp}')
    elif menuFlag is True:
        if menuOptions >= 2:
            menuOptions = 2
        else:
            menuOptions = menuOptions + 1
        print(f'__________________________Rotary Left! New Option : {menuOptions}')

# rotary encoder LEFT
def rotaryLeft():
    global newTemp, menuOptions
    if menuFlag is False:
        if newTemp <= 0:
            newTemp = 0
        else:
            newTemp = newTemp - 1
        print(f'__________________________Rotary Left! New Temp : {newTemp}')
    elif menuFlag is True:
        if menuOptions <= 0:
            menuOptions = 0
        else:
            menuOptions = menuOptions - 1
        print(f'__________________________Rotary Left! New Option : {menuOptions}')
    
rotary = RotaryEncoder(20, 21, bounce_time = bounceTime)
rotary.when_rotated_clockwise = rotaryRight
rotary.when_rotated_counter_clockwise = rotaryLeft

# ----------------------------------------------------------
# rotary encoder push button
def rotaryPressed():
    global menuFlag, menuOptions
    if menuFlag is True:
        if menuOptions == 0:
            if connection is None:
                    connect()
            menuFlag = False
            lcd.clear()
            print(f'__________________________Rotary pressed! Selected :{menuOptions} - Connect')
        if menuOptions == 1:
            # TO-DO
            menuFlag = False
            lcd.clear()
            print(f'__________________________Rotary pressed! Selected :{menuOptions} - Shutdown')
            print(f'__________________________SHUTDOWN INACTIVE!')
        if menuOptions == 2:
            menuFlag = False
            lcd.clear()
            print(f'__________________________Rotary pressed! Selected :{menuOptions} - Exit')
    else:
        return 0

rotarySW = gpiozero.Button(26, bounce_time = bounceTime)
rotarySW.when_pressed = rotaryPressed

# ----------------------------------------------------------
# EMERGENCY button
def emgPressed():
    global emgFlag, menuFlag
    emgFlag = True
    menuFlag = False
    led5.blink(on_time=1, off_time=1)
    stopBtnPressed()
    print(f'__________________________Emergency Button pressed!')

emgButton = gpiozero.Button(12, bounce_time = bounceTime)
emgButton.when_pressed = emgPressed

# ----------------------------------------------------------
# TO-DO: Change to MENU button
# Menu button
def menuPressed():
    global menuFlag
    menuFlag = True
    lcd.clear()

menuBtn = gpiozero.Button(6, bounce_time = bounceTime)
menuBtn.when_pressed = menuPressed

# ----------------------------------------------------------
# Level sensor
def levelMax():
    global maxLevel, levelStr
    if levelSensor.is_active is True:
        led3.blink(on_time=0.5, off_time=0.5)
        maxLevel = True
        levelStr = 'MAX'
    elif levelSensor.is_active is False:
        led3.off()
        maxLevel = False
        levelStr = 'OK'

levelSensor = gpiozero.Button(22)
levelSensor.when_pressed = levelMax
levelSensor.when_released = levelMax

# ----------------------------------------------------------
# set SPI CS pin for channel A
csA = digitalio.DigitalInOut(board.D7)
csA.direction = digitalio.Direction.INPUT
# set SPI CS pin for channel B
csB = digitalio.DigitalInOut(board.D1)
csB.direction = digitalio.Direction.INPUT

# Sensor A initialization
sensorA = adafruit_max31865.MAX31865(spi, 
                                     csA, 
                                     rtd_nominal=100, 
                                     ref_resistor=430.0,
                                     wires=4)
# Sensor B initialization
sensorB = adafruit_max31865.MAX31865(spi,
                                     csB, 
                                     rtd_nominal=100, 
                                     ref_resistor=430.0,
                                     wires=4)


# connect to server function
def connect():
    global connection
    try:
        print(f'Connecting to server... {str(HOST)}:{str(PORT)}')
        connection = mysocket.connect(CONNECTION_QUERY)
        print(CONNECTION_QUERY)
    except:
        print(f'Connection Error.')
        connection = 0
        return connection
    else:
        print(f'Connected to server {str(HOST)}:{str(PORT)}')
        return connection

# send data to server
def sendMeasures(theData):
    if connection != 0:
        try:
            mysocket.emit('measures', theData)
            print(f'Sending ----> {theData}')
    
        except:
            time_stamp = time.strftime('%H:%M:%S')
            print(f'Error! {time_stamp}')
            print(f'Error sending ----> {theData}')
            print('\n')
        
        else:
            time_stamp = time.strftime('%H:%M:%S')
            print(f'Success! {time_stamp}')
            print('\n')
    else:
        time_stamp = time.strftime('%H:%M:%S')
        print('\n')
        print (f'Connection Error! Not connected to server!\n')
        print(f'Error! {time_stamp}')
        print(f'Error sending ----> {theData}')
        printData()


@mysocket.on('connected')
def handle_json(data):
    print(f'Server response : {data}')

@mysocket.on('start')
def startDistil():
    startBtnPressed()
    # TO-DO: Add all the actions
    
@mysocket.on('stop')
def stopDistil():
    stopBtnPressed()
    # TO-DO: Add all the actions

# Add Emergency Button web-socket endpoint
# Add Reset Button web-socket endpoint

def measure():
    global tempA, tempB, resA, resB, now, theData, cpu, s, startStatus
    led2.on()
    led4.on()
    now = time.strftime('%H:%M:%S')
    cpu = CPUTemperature().temperature
    tempA = "{:0.2f}".format(sensorA.temperature)
    resA = "{:0.3f}".format(sensorA.resistance)
    tempB = "{:0.2f}".format(sensorB.temperature)
    resB ="{:0.3f}".format(sensorB.resistance)
    theData = {
            'timeStamp': now,
            'sensorA': [tempA, resA],
            'sensorB': [tempB, resB],
            'status': startStatus,
            'cpuTemp': cpu,
            'userTemp': newTemp,
            'tankLevel': maxLevel
            }
    theData = json.dumps(theData)
    if startStatus is True:
        s = f' < ON   >'
    else:
        s = f' < OFF  >'
    printData()
    printDataLCD()
    printTimeLCDThread()
    sendData()

def sendData():
    sendMeasures(theData)
    led2.off()
    led4.off()

def printDataLCD():
    if menuFlag is False:
        lcd.cursor_pos = (0, 11)
        lcd.write_string(f'{s}')
        lcd.cursor_pos = (1, 0)
        lcd.write_string(f'A : {tempA} C {resA}')
        lcd.cursor_pos = (2, 0)
        lcd.write_string(f'B : {tempB} C {resB}')
        lcd.cursor_pos = (3, 0)
        try:
            lcd.write_string(f'User Temp : {newTemp} ')
        except:
            lcd.write_string(f'User Temp : {newTemp}')
    elif menuFlag is True:
         lcd.cursor_pos = (1, 0)
         lcd.write_string(f'MENU')
         if menuOptions == 0:
             lcd.cursor_pos = (2, 0)
             lcd.write_string(f'Connect ')
         elif menuOptions == 1:
             lcd.cursor_pos = (2, 0)
             lcd.write_string(f'Shutdown')
         elif menuOptions == 2:
             lcd.cursor_pos = (2, 0)
             lcd.write_string(f'Exit    ')

def printTimeLCD():
    lcd.cursor_pos = (0, 0)
    lcd.write_string(f'{time.strftime("%H:%M:%S")}')

# Print data to terminal
def printData():
    print('--------------------------')
    print(f'Last measure  : {now}')
    print(f'CPU           : {cpu} °C')
    print(f'Session ID    : {mysocket.sid}')
    print(f'Sensor A      : {tempA} °C  {resA} Ω')
    print(f'Sensor B      : {tempB} °C  {resB} Ω')
    print(f'Status        : {startStatus}')
    print(f'Tank Level    : {levelStr}')
    try:
        print(f'Threshhold    : {newTemp} °C')
    except:
        print(f'Threshhold    : Reading... °C')
    print('\n')

# Check temperature
def checkTemp():
    if (float(tempA) + 0.5 ) <= newTemp and led1.is_active == True:
        output1.off()
    elif (float(tempA) - 0.5 ) >= newTemp and led1.is_active == True:
        output1.on()

# Set a thread to read the sensors
def readSensorsThread():
    readSensors = threading.Thread(target=measure)
    readSensors.start()

# Set a thread to display time to LCD
def printTimeLCDThread():
    displayTime = threading.Thread(target=printTimeLCD)
    displayTime.start()

lcd.cursor_pos = (1,0)
lcd.write_string(f'Loading...')
lcd.cursor_pos = (2,0)
lcd.write_string(f'Connecting...')
connect()
lcd.clear()
levelMax()
measure()

# Schedule the thread
schedule.every(0.5).seconds.do(readSensorsThread)

try:
    while True:
        checkTemp()
        schedule.run_pending()
        time.sleep(1)
except KeyboardInterrupt:
    print('Exiting operation...')