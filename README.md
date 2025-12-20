---

# Smart Sensor Node for Machine Health Monitoring

## 📌 Overview

This project implements a **Smart Sensor Node for Machine Health Monitoring** using an **STM32 microcontroller**, **TinyML based edge inference**, and **IoT cloud connectivity**. The system monitors key physical parameters of a rotating machine and classifies its health condition locally while optionally publishing summarized data to the cloud.

The project is developed as part of the **CDAC PG DESD final project**.

---

## 🎯 Problem Statement

Conventional machine monitoring systems depend heavily on cloud based processing and continuous data transmission, leading to higher latency, bandwidth usage, and cost.
This project addresses these challenges by performing **local (edge) data processing and TinyML inference**, enabling real time machine health assessment with minimal cloud dependency.

---

## ✨ Key Features

* Multi sensor machine health monitoring:

  * Temperature
  * Current
  * Vibration
  * Speed (RPM)
* Edge based **TinyML inference on STM32**
* Local data logging using **SD card**
* MQTT based cloud communication via **ESP32**
* Cloud integration using **AWS IoT Core**
* Local visualization using **OLED display**
* Modular and scalable system architecture

---

## 🧠 System Architecture

* **STM32F407VGT6**

  * Sensor data acquisition
  * Signal processing & feature extraction
  * TinyML model inference

* **ESP32**

  * WiFi connectivity
  * MQTT communication with cloud

* **Cloud (AWS IoT Core)**

  * Receives machine health status
  * Enables remote monitoring

---

## 🧩 Hardware Components

* STM32F407VGT6 Development Board
* ESP32 as WiFi Module
* Temperature Sensor
* Current Sensor
* Vibration Sensor
* RPM / Speed Sensor
* OLED Display
* SD Card Module
* Rotating Machine (Table Fan / Motor)

---

## 🛠 Software & Tools

* STM32CubeIDE
* Arduino IDE (ESP32)
* TinyML framework (TensorFlow Lite Micro or equivalent)
* MQTT Protocol
* AWS IoT Core
* FAT File System (SD Card)

---

## 📂 Project Structure

```
├── stm32_firmware/
│   ├── sensor_drivers/
│   ├── tinyml_model/
│   ├── sdcard_logging/
│   └── main.c
│
├── esp32_firmware/
│   ├── mqtt_client/
│   └── wifi_config/
│
├── data/
│   ├── raw_sensor_logs/
│   └── training_dataset/
│
├── docs/
│   ├── block_diagram.png
│   └── flow_chart.png
│
└── README.md
```

---

## 🚀 How It Works

1. Sensors continuously collect machine parameters
2. STM32 performs preprocessing and feature extraction
3. TinyML model classifies machine health locally
4. Data is logged on SD card for offline analysis
5. Health status is published to AWS IoT via MQTT using ESP32
6. OLED displays real time machine condition

---

## 📊 Machine Health States

* **Normal**
* **Warning**
* **Fault**

---

## ⚠️ Limitations

* Prototype level system (not industrial certified)
* Limited dataset for ML training
* Designed for single rotating machine
* No closed loop control or automatic shutdown

---

## 🔮 Future Scope

* Support for multiple machines
* Advanced ML models
* Mobile application for alerts
* Industrial grade sensors and enclosure

---

## 👥 Team

* **Rohit Bhoge**           `Team Member (CDAC PG DESD, ACTS Pune)`
* **Navin Bharadwaj**       `Team Member (CDAC PG DESD, ACTS Pune)`
* **Ajinkya Sawant**        `Team Member (CDAC PG DESD, ACTS Pune)`
* **Sandesh Soni**          `Team Member (CDAC PG DESD, ACTS Pune)`
* **Samriddh Shrivastav**   `Team Member (CDAC PG DESD, ACTS Pune)`
---

## 📜 License

This project is developed for academic purposes.
Feel free to fork and modify for learning and research.

---
