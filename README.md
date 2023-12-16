# PID Ball Balancing System

## 🌟 Introduction

This project develops a ball stability control system using a PID controller. It is designed to provide practical
understanding and hands-on experience with PID controllers, integrating hardware with software for real-world
application.

## ⚙️ Hardware and Software Requirements

### Hardware:

- Arduino Microcontroller (Atmega328P)
- ESP32 module for WiFi connectivity
- Ultrasonic Sensor
- Servo Motor
- Stable Platform
- Power Supply

### Software:

- Microchip Studio (Atmel Studio) for programming the Arduino microcontroller
- Arduino IDE for programming the ESP32 module

## 🚀 Setup Guide

### Hardware Setup

1. Assemble the platform with the servo motor and ultrasonic sensor.
2. Connect the Arduino Microcontroller to the servo motor and ultrasonic sensor.
3. Ensure the ESP32 module is wired for WiFi connectivity.
4. Verify the power supply outputs using a multimeter.

Please refer to the [circuit diagram](circuit-diagram.jpg) provided for detailed wiring information.

### Software Setup

#### For Arduino Atmega328P:

1. Create a new project in Microchip Studio (Atmel Studio) and include `uart.h`, `uart.c`, and `main.c`.
2. Compile the project and upload it to the Arduino board.

#### For ESP32 Feather V2:

1. Open `blynk.ino` in the Arduino IDE.
2. Compile and upload the code to the ESP32 module.

### Running the Project

1. Power up the system and ensure that the servo motor adjusts the platform to stabilize the ball at the center.
2. Use the BLYNK app to wirelessly control and adjust PID values from a mobile device.

## 🛠️ Verification and Testing

- **Hardware Testing:**
    - Test each hardware component individually for proper operation.
    - Ensure the system as a whole operates cohesively.

- **Software Testing:**
    - Implement and compile the PID library in the microcontroller.
    - Calibrate the sensor for accurate distance measurements.
    - Test wireless communication of PID values from the mobile device.

## 🎉 Results

The project successfully stabilized the ball on the platform, demonstrating the PID controller's efficacy. Challenges
included tuning PID values and integral overflow issues, which were addressed during the project.

## 🔮 Demos

Check out more about our project on
our [Devpost](https://devpost.com/software/ball-stability-control-using-pid-controller) page!

[Here](https://www.youtube.com/watch?v=rWQ8Ez3nQEM) is a demo video of our working system.

[Here](https://youtu.be/p7XoBY4ouEg?si=I2p8ivDkeHnvJMq5) is a demo video of using wireless control to set PID Values
from a mobile device.

## 🪄 Conclusion

This project provided valuable insights into control systems, software development, and system integration, proving the
effectiveness of a well-tuned PID control system.

## 💡 Further Improvements

- Implementing more sophisticated hardware for precision control.
- Adding an LCD display for real-time PID value monitoring.
- Scaling the project for more complex applications like robotic movement control.