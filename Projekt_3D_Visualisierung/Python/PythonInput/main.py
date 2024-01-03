import serial
import json
import csv
from datetime import datetime

def main():
    com_port = 'COM10'
    baud_rate = 115200
    ser = serial.Serial(com_port, baud_rate)

    while True:
        input_line = ser.readline().decode('utf-8').strip()
        if not input_line:
            continue  # Skip empty line
        try:
            data = json.loads(input_line)
            if "euler" in data.keys():
                print(data["euler"])
                euler(data)
            if "quaternions" in data.keys():
                print(data["quaternions"])
                quaternions(data)

        except json.JSONDecodeError as e:
            print(f"Fehler beim Dekodieren der JSON-Daten: {e}")


def euler(data):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    with open('euler_data.csv', 'a', newline='') as csvfile:
        fieldnames = ['Timestamp', 'Euler_X', 'Euler_Y', 'Euler_Z', 'Acc_X', 'Acc_Y', 'Acc_Z', 'Gyr_X', 'Gyr_Y',
                      'Gyr_Z', 'Mag_X', 'Mag_Y', 'Mag_Z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if csvfile.tell() == 0:  # Check if the file is empty, write header if true
            writer.writeheader()

        writer.writerow({
            'Timestamp': timestamp,
            'Euler_X': data['euler'][0],
            'Euler_Y': data['euler'][1],
            'Euler_Z': data['euler'][2],
            'Acc_X': data['acc'][0],
            'Acc_Y': data['acc'][1],
            'Acc_Z': data['acc'][2],
            'Gyr_X': data['gyr'][0],
            'Gyr_Y': data['gyr'][1],
            'Gyr_Z': data['gyr'][2],
            'Mag_X': data['mag'][0],
            'Mag_Y': data['mag'][1],
            'Mag_Z': data['mag'][2]
        })

def quaternions(data):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

    with open('quaternion_data.csv', 'a', newline='') as csvfile:
        fieldnames = ['Timestamp', 'Quaternion_X', 'Quaternion_Y', 'Quaternion_Z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        if csvfile.tell() == 0:  # Check if the file is empty, write header if true
            writer.writeheader()

        writer.writerow({
            'Timestamp': timestamp,
            'Quaternion_X': data['quaternions'][0],
            'Quaternion_Y': data['quaternions'][1],
            'Quaternion_Z': data['quaternions'][2]
        })

if __name__ == '__main__':
    main()
