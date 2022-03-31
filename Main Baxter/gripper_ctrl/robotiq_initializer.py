import serial
import time
import binascii


def robotiq_initialize():
    serL = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    counter = 0
    while counter < 1:
        counter = counter + 1
        serL.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
        data_raw = serL.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        time.sleep(0.01)

        serL.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
        data_raw = serL.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        time.sleep(0.5)

    if True:
        # print("Close gripper")
        serL.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        time.sleep(0.5)

        # print("Open gripper")
        serL.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")


    serR = serial.Serial(port='/dev/ttyUSB1', baudrate=115200, timeout=1, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    counter = 0
    while counter < 1:
        counter = counter + 1
        serR.write("\x09\x10\x03\xE8\x00\x03\x06\x00\x00\x00\x00\x00\x00\x73\x30")
        data_raw = serR.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        time.sleep(0.01)

        serR.write("\x09\x03\x07\xD0\x00\x01\x85\xCF")
        data_raw = serR.readline()
        print(data_raw)
        data = binascii.hexlify(data_raw)
        time.sleep(0.5)

    if True:
        # print("Close gripper")
        serR.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\xFF\xFF\xFF\x42\x29")
        time.sleep(0.5)

        # print("Open gripper")
        serR.write("\x09\x10\x03\xE8\x00\x03\x06\x09\x00\x00\x00\xFF\xFF\x72\x19")

    return serL, serR

if __name__ == "__main__":
    robotiq_initialize()
