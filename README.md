# Animal Health Tracker

Smart wearable collar for livestock health monitoring and tracking.

## 🚀 Features
- Real-time body temperature monitoring
- Activity tracking using motion sensor
- GPS-based location tracking
- Cloud-based visualization using Blynk IoT
- Portable and low-power wearable design

## 🧠 System Architecture

The system is built around an ESP32 NodeMCU which acts as the main controller.  
It collects data from multiple sensors, processes it, and transmits it to the cloud.

**Data Flow:**
1. Sensors (Temperature, Motion, GPS) collect real-time data
2. ESP32 processes and filters the sensor data
3. Data is transmitted via Wi-Fi to Blynk IoT Cloud
4. User monitors live data through the Blynk mobile application

## 📌 Overview
This project uses an ESP32 NodeMCU to monitor animal health parameters like body temperature, activity, and location.  
The data is displayed in real time using the Blynk IoT platform.

## 🛠️ Hardware Used
- ESP32 NodeMCU
- DS18B20 Temperature Sensor
- MPU-6050 Accelerometer & Gyroscope
- NEO-6M GPS Module
- MicroSD Card Module
- 3.7V Li-Po Battery + TP4056 Charger

## 🌐 Software
- Arduino IDE
- Blynk IoT Platform

## 📂 Repository Structure
- hardware/ → Circuit & block diagrams  
- firmware/ → ESP32 source code  
- images/ → Prototype and app screenshots  
- docs/ → Project documentation


