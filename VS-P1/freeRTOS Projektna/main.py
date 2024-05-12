import serial
import struct


def main():
    ser = serial.Serial('COM9', baudrate=115200) #timeout=1

    while True:
        packed_data = ser.read(8)
            
            # dobimo struct od stm32
        unpaked_data = struct.unpack('<4h', packed_data)

        if unpaked_data[0] == -21846 : 
            print("Gyro: ",unpaked_data[1], unpaked_data[2], unpaked_data[3])
        elif unpaked_data[0] == -17477:
            print("Accel: ",unpaked_data[1], unpaked_data[2], unpaked_data[3])

if __name__ == "__main__":
    main()