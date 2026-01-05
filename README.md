# Animal-Health-Tracker-
IoT-based animal health monitoring system using ESP32

# Animal Health Tracker – Smart Collar for Livestock Monitoring

## Overview
A wearable IoT-based system to monitor animal health parameters such as body temperature, activity, and location in real time.

## Hardware Components
- ESP32 NodeMCU (Main Controller)
- DS18B20 Temperature Sensor
- MPU-6050 Accelerometer & Gyroscope
- NEO-6M GPS Module
- microSD Card Module
- 3.7V Li-Po Battery
- TP4056 Charging Module

## Power Management
The system is powered using a 3.7V Li-Po battery.  
A TP4056 module handles safe charging, and the ESP32 provides regulated 3.3V supply to sensors.

## Working Principle
- ESP32 collects data from all sensors
- Temperature and motion data indicate animal health and activity
- GPS module provides real-time location
- Data is sent to Blynk IoT platform via Wi-Fi
- If Wi-Fi is unavailable, data is stored locally on SD card
- Stored data is uploaded once connectivity is restored

## Mobile Application (Blynk)
The Blynk app displays:
- Body temperature graph
- Motion/activity status
- GPS location on map
- Health alerts (fever, inactivity)

## Applications
- Livestock health monitoring
- Early detection of illness
- Remote tracking of animals
