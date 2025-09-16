# About this repository

This repository contains the firmware developed as part of a thesis project at **Linnaeus University (LNU)**.  
The firmware powers a monitoring prototype designed for collecting and transmitting environmental and positional data.  

## Repository structure
- **Components/** – Libraries and reusable components shared across the project.  
- **Main/** – The core application, organized into modules that encapsulate specific functionalities.  

## Features
- **VOC Monitoring** – Measures air quality data using the **Bosch BME690** sensor.  
  - Heating profiles can be configured by supplying a `.bmeconfig` file in `main/littlefs/`.  
- **GPS Positioning** – Parses **NMEA sentences** from the **Quectel LC76G(AB)** module to retrieve positional information.  
- **Motion Sensing** – Samples data from an **ADXL345 accelerometer** to support fall detection  
  *(note: no dedicated fall detection algorithm is included).*  
- **Data Transmission** – Sends data via **MQTT over Wi-Fi Access Point**.  
  - The MQTT client requires `caRoot.pem`, `client.crt`, and `client.key` for mutual TLS (mTLS).  
