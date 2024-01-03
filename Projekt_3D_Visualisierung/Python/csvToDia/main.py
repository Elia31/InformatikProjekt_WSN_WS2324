import pandas as pd
import matplotlib.pyplot as plt

# Daten aus der CSV-Datei lesen
df = pd.read_csv('euler_data.csv', parse_dates=['Timestamp'])

# Plot für Euler_X, Euler_Y und Euler_Z erstellen
plt.figure(figsize=(10, 6))
plt.plot(df['Timestamp'], df['Euler_X'], label='Euler_X')
plt.plot(df['Timestamp'], df['Euler_Y'], label='Euler_Y')
plt.plot(df['Timestamp'], df['Euler_Z'], label='Euler_Z')
plt.title('Euler Angles Over Time')
plt.xlabel('Timestamp')
plt.ylabel('Angle (degrees)')
plt.legend()
plt.show()

# Plot für Acc_X, Acc_Y und Acc_Z erstellen
plt.figure(figsize=(10, 6))
plt.plot(df['Timestamp'], df['Acc_X'], label='Acc_X')
plt.plot(df['Timestamp'], df['Acc_Y'], label='Acc_Y')
plt.plot(df['Timestamp'], df['Acc_Z'], label='Acc_Z')
plt.title('Acceleration Over Time')
plt.xlabel('Timestamp')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()
plt.show()

# Plot für Gyr_X, Gyr_Y und Gyr_Z erstellen
plt.figure(figsize=(10, 6))
plt.plot(df['Timestamp'], df['Gyr_X'], label='Gyr_X')
plt.plot(df['Timestamp'], df['Gyr_Y'], label='Gyr_Y')
plt.plot(df['Timestamp'], df['Gyr_Z'], label='Gyr_Z')
plt.title('Gyroscope Readings Over Time')
plt.xlabel('Timestamp')
plt.ylabel('Angular Velocity (deg/s)')
plt.legend()
plt.show()

# Plot für Mag_X, Mag_Y und Mag_Z erstellen
plt.figure(figsize=(10, 6))
plt.plot(df['Timestamp'], df['Mag_X'], label='Mag_X')
plt.plot(df['Timestamp'], df['Mag_Y'], label='Mag_Y')
plt.plot(df['Timestamp'], df['Mag_Z'], label='Mag_Z')
plt.title('Magnetometer Readings Over Time')
plt.xlabel('Timestamp')
plt.ylabel('Magnetic Field Strength (uT)')
plt.legend()
plt.show()

