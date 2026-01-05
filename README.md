# Animal Health Tracker

Smart wearable collar for livestock health monitoring and tracking.

## 📌 Overview
This project uses an ESP32 NodeMCU to monitor animal health parameters like body temperature, activity, and location.  
The data is displayed in real time using the Blynk IoT platform.
This system aims to assist farmers in early disease detection and livestock management through continuous health monitoring.

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

## ⚙️ Working Principle

1. The ESP32 NodeMCU powers up and initializes all connected sensors.
2. The DS18B20 sensor measures the animal’s body temperature.
3. The MPU-6050 tracks movement and activity to analyze animal behavior.
4. The NEO-6M GPS module provides real-time location coordinates.
5. All sensor data is processed by the ESP32 and sent to the Blynk IoT cloud via Wi-Fi.
6. The farmer can monitor temperature, activity, and location through the Blynk mobile app.
7. 7. In case of Wi-Fi unavailability, data can be stored locally on a microSD card for later retrieval.

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

## 🚀 Future Scope

- Integration of additional health sensors such as heart rate and respiration sensors
- AI-based anomaly detection to identify early signs of illness
- GSM / LoRaWAN connectivity for remote areas without Wi-Fi
- Solar-powered charging for long-term outdoor deployment
- Mobile alerts and notifications for abnormal health conditions
- Data analytics dashboard for long-term health trend analysis

## 📂 Repository Structure
- hardware/ → Circuit & block diagrams  
- firmware/ → ESP32 source code  
- images/ → Prototype and app screenshots  
- docs/ → Project documentation


