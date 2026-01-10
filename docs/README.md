## Project Documentation

This folder contains application details, advantages, and future scope.

🔹 System Flowchart Explanation

Explanation:

The system starts by collecting data from the sensors attached to the animal.
The temperature sensor measures body temperature and the motion sensor detects movement and activity.

All sensor data is sent to the ESP32 microcontroller, which acts as the main controller of the system.

The GPS module continuously provides location coordinates (latitude and longitude) to the ESP32.

The ESP32 stores sensor readings and location data locally in the microSD card for backup and offline access.

The processed data is transmitted through the communication module (Wi-Fi / GSM / LoRa).

Data is displayed on the mobile application using the Blynk platform.

The cloud (Blynk server) stores real-time data and allows remote monitoring by the user.

If abnormal conditions are detected (high temperature, abnormal motion, or geofence breach), alerts are generated for user monitoring.
