# Project Overview  
## Smart Sensor Node for Machine Health

---

## 1. Introduction

In modern industrial environments, unexpected machine failures can lead to high downtime, maintenance costs, and safety risks. Traditional maintenance techniques such as periodic inspection or reactive repair are inefficient and often fail to detect early-stage faults.

This project, **Smart Sensor Node for Machine Health**, focuses on developing a low-cost, edge-based predictive maintenance system that continuously monitors machine health using electrical current and vibration signals. By combining embedded systems with machine learning, the system detects anomalies in real time and enables condition-based maintenance.

---

## 2. Problem Statement

Industrial machines often fail due to:
- Bearing wear
- Mechanical imbalance
- Electrical overload
- Misalignment

Conventional monitoring systems:
- Are expensive
- Require cloud dependency
- Do not provide real-time edge intelligence

Hence, there is a need for an **intelligent, standalone, edge-deployable sensor node** capable of detecting abnormal machine behavior early.

---

## 3. Proposed Solution

The proposed system uses:
- **Current sensing** to monitor electrical load variations
- **Vibration sensing** to capture mechanical abnormalities
- **Unsupervised machine learning (Autoencoder)** for anomaly detection
- **Edge inference on STM32**, avoiding cloud dependency
- **ESP32-based communication** for remote monitoring

The system learns the normal operating behavior of a machine and flags deviations as potential faults.

---

## 4. System Architecture

### High-Level Flow:
1. Sensors acquire current and vibration data
2. STM32 performs signal processing and feature extraction
3. Autoencoder model evaluates machine health
4. Health score and anomaly status are generated
5. ESP32 transmits data to dashboard/cloud

---

## 5. Hardware Components

| Component | Description |
|---------|-------------|
| STM32 | Core controller for data acquisition and ML inference |
| ACS712 | Hall-effect based current sensor |
| ADXL345 | 3-axis digital vibration sensor |
| ESP32 | Wireless communication module |
| Power Supply | Regulated 3.3V and 5V rails |

---

## 6. Software Components

### Embedded Firmware
- Sensor interfacing (ADC, SPI)
- RMS and statistical feature extraction
- Autoencoder inference
- UART communication with ESP32

### Machine Learning
- Autoencoder trained on healthy machine data
- Reconstruction error-based anomaly detection
- Lightweight model optimized for embedded deployment

### Dashboard
- Real-time visualization of machine health
- Historical data logging
- Fault indication alerts

---

## 7. Machine Learning Approach

- **Type**: Unsupervised Learning
- **Model**: Autoencoder
- **Training Data**: Healthy machine operation only
- **Inference Metric**: Reconstruction Error

If reconstruction error exceeds a predefined threshold, the machine is classified as **unhealthy**.

---

## 8. Key Features

- Edge-based intelligence (no continuous cloud dependency)
- Real-time fault detection
- Low power and low cost
- Scalable for multiple machines
- Suitable for industrial IoT applications

---

## 9. Applications

- Predictive maintenance
- Industrial condition monitoring
- Motor health monitoring
- Smart factories
- Industry 4.0 systems

---

## 10. Conclusion

The Smart Sensor Node for Machine Health demonstrates how embedded systems and machine learning can be effectively combined to create an intelligent, real-time, and scalable predictive maintenance solution. By performing inference directly on the edge, the system ensures faster response, improved reliability, and reduced operational costs.

---

## 11. Future Enhancements

- FFT-based frequency domain analysis
- Multi-class fault classification
- OTA firmware updates
- Edge-to-cloud analytics
- Deployment on multiple sensor nodes
