import serial

with serial.Serial('/dev/cu.wchusbserialfa130', 9600, timeout=10) as ser:
    while True:
        servoAngle = input('What do you want the servo angle? (0-180) ')

        servoAngle = servoAngle + '\n'
        print(servoAngle)
        ser.write(bytes(servoAngle, 'utf-8'))
