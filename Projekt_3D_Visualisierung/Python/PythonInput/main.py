import serial
import json
import csv
import math
import time
import numpy as np
from datetime import datetime


class BNO055Data:
    def __init__(self):
        self.thetaM = 0.0
        self.phiM = 0.0
        self.thetaFold = 0.0
        self.thetaFnew = 0.0
        self.phiFold = 0.0
        self.phiFnew = 0.0
        self.thetaG = 0.0
        self.phiG = 0.0
        self.theta = 0.0
        self.phi = 0.0
        self.thetaRad = 0.0
        self.phiRad = 0.0
        self.Xm = 0.0
        self.Ym = 0.0
        self.psi = 0.0
        self.dt = 0.0
        self.millisOld = int(time.time() * 1000)


bno055 = BNO055Data()
toRad = 2 * math.pi / 360
toDeg = 1 / toRad


def main():
    #com_port = input("Geben Sie den gewünschten COM-Port ein (z.B. 10 für COM Port 10): ")
    #com_port = "COM" + com_port
    com_port = "COM10"
    baud_rate = 115200
    ser = serial.Serial(com_port, baud_rate)

    while True:
        input_line = ser.readline().decode('utf-8').strip()
        try:
            data = json.loads(input_line)
            if "euler" in data.keys():
                euler = np.array(data['euler'])
                #print("Euler before:", euler)
                euler = calc_euler(euler)
                #print("Roll=", euler[0] * toDeg, " Pitch=", euler[1] * toDeg, "Yaw=", euler[2] * toDeg)
                #print("Euler after:", euler)
                #write_euler(euler)
            elif "quaternions" in data.keys():
                quat = np.array(data['quaternions'])
                #write_quat(quat)

        except json.JSONDecodeError as e:
            print(f"Fehler beim Dekodieren der JSON-Daten: {e}")


def calc_euler(data) -> np.array:
    # sprintf(buffer, "{\"euler\": [%d, %d, %d, %d, %d, %d, %d, %d, %d]}", bno055.acc_x, bno055.acc_y, bno055.acc_z,
    # bno055.gyr_x, bno055.gyr_y, bno055.gyr_z, bno055.mag_x, bno055.mag_y, bno055.mag_z);
    #     012      345      678
    # acc xyz, gyr xyz, mag xyz

    # accelerometer unit:               1 m/s^2 = 100 LSB
    # gyroscope unit:                   1 dps (degree per second) = 16 LSB
    # magnetometer unit:                1 mT = 16 LSB

    accX = data[0] / 100
    accY = data[1] / 100
    accZ = data[2] / 100
    gyrX = (data[3] / 16) * 0.01745329251 # direkt zu rps weil adruino standard in rps ist
    gyrY = (data[4] / 16) * 0.01745329251 # direkt zu rps weil adruino standard in rps ist
    gyrZ = (data[5] / 16) * 0.01745329251 # direkt zu rps weil adruino standard in rps ist
    magX = data[6] / 16
    magY = data[7] / 16
    magZ = data[8] / 16

    # https://toptechboy.com/9-axis-imu-lesson-20-vpython-visualization-of-roll-pitch-and-yaw/
    bno055.thetaM = -(math.atan2(accX / 9.81, accZ / 9.81)) / 2 / math.pi * 360
    bno055.phiM = -(math.atan2(accY / 9.81, accZ / 9.81)) / 2 / math.pi * 360
    bno055.phiFnew = 0.95 * bno055.phiFold + 0.05 * bno055.phiM
    bno055.thetaFnew = 0.95 * bno055.thetaFold + 0.05 * bno055.thetaM

    bno055.dt = (int(time.time() * 1000) - bno055.millisOld) / 1000
    bno055.millisOld = int(time.time() * 1000)

    bno055.theta = (bno055.theta + gyrY * bno055.dt) * 0.95 + bno055.thetaM * 0.05
    bno055.phi = (bno055.phi - gyrX * bno055.dt) * 0.95 + bno055.phiM * 0.05
    bno055.thetaG = bno055.thetaG + gyrY * bno055.dt
    bno055.phiG = bno055.phiG - gyrX * bno055.dt

    bno055.phiRad = bno055.phi / 360 * (2 * math.pi)
    bno055.thetaRad = bno055.theta / 360 * (2 * math.pi)

    bno055.Xm = magX * math.cos(bno055.thetaRad) - magY * math.sin(bno055.phiRad) * math.sin(bno055.thetaRad) + magZ * math.cos(bno055.phiRad) * math.sin(bno055.thetaRad)
    bno055.Ym = magY * math.cos(bno055.phiRad) + magZ * math.sin(bno055.phiRad)

    bno055.psi = math.atan2(bno055.Ym, bno055.Xm) / (2 * math.pi) * 360

    bno055.phiFold = bno055.phiFnew
    bno055.thetaFold = bno055.thetaFnew

    #print("Acc X Y Z: ", accX, accY, accZ)
    #print("Gyr X Y Z: ", gyrX, gyrY, gyrZ)
    #print("Mag X Y Z: ", magX, magY, magZ)
    #print("thetaM = ", bno055.thetaM, "\nphiM = ", bno055.phiM, "\nphiFnew = ", bno055.phiFnew, "\nthetaFnew = ", bno055.thetaFnew)
    #print("dt = ", bno055.dt, "\nmillisOld = ", bno055.millisOld)
    #print("theta = ", bno055.theta, "\nphi = ", bno055.phi, "\nthetaG = ", bno055.thetaG, "\nphiG = ", bno055.phiG)
    #print("phiRad = ", bno055.phiRad, "\nthetaRad = " , bno055.thetaRad)
    #print("Xm = ", bno055.Xm, "\nYm = ", bno055.Ym)
    #print("psi = ", bno055.psi)

    # phi = roll, theta = pitch, psi = yaw

    #  0     1    2       3 4 5      6 7 8      9 10 11
    # roll pitch yaw, acc x y z, gyr x y z, mag x y  z
    # roll, pitch, yaw, accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ

    return np.array([bno055.phi, bno055.theta, bno055.psi, accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ])


def write_euler(data):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    with open('euler_data.csv', 'a', newline='') as csvfile:
        fieldnames = ['Timestamp', 'Euler_Roll', 'Euler_Pitch', 'Euler_Yaw', 'Acc_X', 'Acc_Y', 'Acc_Z', 'Gyr_X', 'Gyr_Y', 'Gyr_Z',
                       'Mag_X', 'Mag_Y', 'Mag_Z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if csvfile.tell() == 0:  # Check if the file is empty, write header if true
            writer.writeheader()

        #        0     1    2       3 4 5      6 7 8      9 10 11
        # euler roll pitch yaw, acc x y z, gyr x y z, mag x y z
        writer.writerow({
            'Timestamp': timestamp,
            'Euler_Roll': data[0],
            'Euler_Pitch': data[1],
            'Euler_Yaw': data[2],
            'Acc_X': data[3],
            'Acc_Y': data[4],
            'Acc_Z': data[5],
            'Gyr_X': data[6],
            'Gyr_Y': data[7],
            'Gyr_Z': data[8],
            'Mag_X': data[9],
            'Mag_Y': data[10],
            'Mag_Z': data[11]
        })


def write_quat(data):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    with open('quaternion_data.csv', 'a', newline='') as csvfile:
        fieldnames = ['Timestamp', 'Quaternion_W', 'Quaternion_X', 'Quaternion_Y', 'Quaternion_Z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if csvfile.tell() == 0:  # Check if the file is empty, write header if true
            writer.writeheader()
        # quat w x y z
        writer.writerow({
            'Timestamp': timestamp,
            'Quaternion_W': data[0],
            'Quaternion_X': data[1],
            'Quaternion_Y': data[2],
            'Quaternion_Z': data[3]
        })


if __name__ == '__main__':
    main()
