# Features and Limitations

**Smart Sensor Node for Machine Health Monitoring**

---

## 1. Features

### 1.1 Multi Sensor Machine Health Monitoring

The system monitors multiple physical parameters that are critical for assessing the health of rotating machinery.

**Monitored Parameters:**

* **Temperature**
  → Used for detecting overheating conditions
* **Current**
  → Indicates load and overload conditions
* **Vibration**
  → Used to detect bearing faults and mechanical imbalance
* **Speed (RPM)**
  → Helps identify mechanical degradation and abnormal operation

---

### 1.2 Edge Processing using STM32F407

The STM32F407 microcontroller performs all real time data processing locally at the edge.

**Key Functions:**

* Sensor data acquisition
* Signal conditioning and filtering
* Feature extraction (RMS,Mean,Peak,Variance)

This approach reduces raw data transmission and improves real time responsiveness.

---

### 1.3 TinyML Based Local Fault Inference

A lightweight TinyML model is deployed directly on the STM32 microcontroller.

**Capabilities:**

* Detects machine operating states:
  * **Normal**
  * **Warning**
  * **Fault**
* No continuous cloud dependency for decision making
* Enables low latency, real time fault detection at the edge

---

### 1.4 SD Card Based Data Logging

The system includes local storage using an SD card.

**Usage:**

* Offline analysis of sensor data
* Dataset collection for training and validating the TinyML model

This supports a practical data driven machine learning workflow.

---

### 1.5 Hybrid Communication Architecture

A dual controller architecture is used to separate processing and communication tasks.

**Architecture Details:**

* **ESP32** is used exclusively for:
  * WiFi connectivity
  * MQTT communication
* **STM32 <-> ESP32 communication** via UART or SPI
* MQTT data published to **AWS IoT Core**, including:
  * Machine health status
  * Alerts
  * Aggregated sensor metrics

---

### 1.6 Cloud Visualization and Alerts

The cloud layer provides remote monitoring and historical insights.

**Features:**

* Real time machine health status
* Timestamped health events
* Predictive maintenance alerts
* Historical trend observation

Only high level information is transmitted, not raw sensor data.

---

### 1.7 OLED Based Local Display

The system includes an OLED display for standalone operation.

**Displayed Information:**

* Live sensor values
* Current machine health state
* Fault indication

This ensures usability even without cloud connectivity.

---

### 1.8 Predictive Maintenance Use Case

The project is designed specifically for predictive maintenance of rotating machinery.

**Applicable Machines:**

* Fan
* Motor
* Pump

The focus is on early fault detection rather than corrective control.

---

## 2. Limitations

### 2.1 WiFi Dependency for Cloud Features

Cloud based monitoring depends on ESP32 WiFi connectivity.

* If WiFi is available:
  * Cloud visualization and alerts function normally
* If WiFi fails:
  * Local TinyML inference continues to operate
  * Cloud visualization and updates stop

---

### 2.2 No Closed Loop Control

The system is designed strictly for monitoring and diagnostics.

**Not Supported:**

* Automatic machine shutdown
* Actuator or relay control

The system only generates alerts and health insights.

---

### 2.3 Limited Sensor Accuracy

Cost effective sensors are used to maintain feasibility.

* Not laboratory grade or calibration grade sensors
* Accuracy is sufficient for:
  * Trend detection
  * Fault classification
* Not suitable for precision measurement applications

---

### 2.4 Power Consumption Not Optimized

Power optimization is not a primary design objective.

* No sleep modes implemented
* No battery powered operation
* Intended for continuously powered environments

---

### 2.5 Limited Dataset for ML Training

The TinyML model is trained on a small, controlled dataset.

* Dataset size is limited
* Focus is on:
  * Methodology
  * End to end workflow
* Not aimed at large scale industrial generalization

---

### 2.6 Limited Machine Types

The system is validated on a single rotating machine.

* Example machines:
  * Table fan
  * Motor
* Not designed for multi machine or complex industrial setups

---

### Scope Clarification Statement

> This project focuses on demonstrating an end to end architecture for edge based machine health monitoring using TinyML and IoT. The emphasis is on system design, data flow, and predictive maintenance methodology rather than industrial deployment or large scale datasets.

---
