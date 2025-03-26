# MPU6050/9250 IMU Sensor Interface

This Python script provides an interface for the MPU6050/9250 series Inertial Measurement Unit (IMU) sensors. It retrieves gyroscope and accelerometer data and processes it using a complementary filter to obtain Roll, Pitch, and Yaw values.

## Features

- Interfaces with MPU6050/9250 IMU sensors
- Retrieves raw gyroscope and accelerometer data
- Calibrates the gyroscope
- Processes data using a complementary filter
- Calculates Roll, Pitch, and Yaw angles
- Provides acceleration magnitude

## Requirements

- Python 3.x
- `smbus` library
- `numpy` library

## Installation

1. Clone this repository or download the script.
2. Install the required libraries:

```
pip install smbus numpy
```

## Usage

1. Connect the IMU sensor to I2C bus 1 of your device.
2. Import the `MPU` class from the script:

```python
from mpu_sensor import MPU
```

3. Create an instance of the `MPU` class:

```python
gyro = 250  # Gyroscope sensitivity: 250, 500, 1000, or 2000 [deg/s]
acc = 4     # Accelerometer sensitivity: 2, 4, 7, or 16 [g]
tau = 0.9   # Complementary filter coefficient
mpu = MPU(gyro, acc, tau)
```

4. Set up the sensor and calibrate the gyroscope:

```python
mpu.setUp()
mpu.calibrateGyro(500)  # Calibrate with 500 points
```

5. Use the `compFilter()` method to get processed data:

```python
angles, accelerations = mpu.compFilter()
roll, pitch, yaw = angles
accel_x, accel_y, accel_z, accel_magnitude = accelerations
```

## Example

```python
import time
from mpu_sensor import MPU

def main():
    mpu = MPU(250, 4, 0.9)
    mpu.setUp()
    mpu.calibrateGyro(500)

    start_time = time.time()
    while time.time() < (start_time + 20):  # Run for 20 seconds
        angles, accelerations = mpu.compFilter()
        print(f"Roll: {angles[0]:.2f}, Pitch: {angles[1]:.2f}, Yaw: {angles[2]:.2f}")
        print(f"Accel X: {accelerations[0]:.2f}, Y: {accelerations[1]:.2f}, Z: {accelerations[2]:.2f}, Magnitude: {accelerations[3]:.2f}")

if __name__ == '__main__':
    main()
```

## Output

The `compFilter()` method returns two lists:

1. `[roll, pitch, yaw]`: Angles in degrees
2. `[accel_x, accel_y, accel_z, accel_magnitude]`: Acceleration values and magnitude

## License

This project is open-source and available under the MIT License.

## Contributing

Contributions, issues, and feature requests are welcome. Feel free to check [issues page](https://github.com/yourusername/mpu-sensor-interface/issues) if you want to contribute.

## Author
Original Author : https://github.com/MarkSherstan/MPU-6050-9250-I2C-CompFilter
i added human-readable angle,accel calculation features.
