import serial
import struct
import time


def main():
    ser = serial.Serial('COM3', baudrate=115200) #timeout=1

    while True:
        packed_data = ser.read(20)

        unpacked_data = struct.unpack('<10h', packed_data)
        #unpacked_data = struct.unpack('<5f', packed_data)

        #print(unpacked_data)

        

        #print(unpacked_data[0])
        if unpacked_data[0] == -21846:
            #unpacked_data = struct.unpack('<10h', packed_data)
            print("Gyro: ",unpacked_data[1], unpacked_data[2], unpacked_data[3])
        elif unpacked_data[0] == -17664:
            unpacked_data = struct.unpack('<5f', packed_data)
            print("Accel: ", unpacked_data[1], unpacked_data[2], unpacked_data[3], "Clock:", unpacked_data[4])
        # else:
        #     print("Nekaj narobe!")
        #     print(head)
        #     return 0
        time.sleep(0.1)
        

            
            # dobimo struct od stm32
        #unpaked_data = struct.unpack('<4h', packed_data)
        

        # if unpaked_data[0] == -21846 : 
        #     print("Gyro: ",unpaked_data[1], unpaked_data[2], unpaked_data[3])
        # elif unpaked_data[0] == -17477:
        #     print("Accel: ",unpaked_data[1], unpaked_data[2], unpaked_data[3])
        #print("Accel: ",unpaked_data[0], unpaked_data[1], unpaked_data[2])


if __name__ == "__main__":
    main()