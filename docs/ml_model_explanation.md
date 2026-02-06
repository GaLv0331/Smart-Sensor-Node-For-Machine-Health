# Machine Learning Overview  
## Smart Sensor Node for Machine Health

---

## 1. Introduction

The machine learning component of this project is designed to enable **predictive maintenance** by detecting abnormal machine behavior using sensor data. The system uses an **unsupervised learning approach** so that it can learn normal machine behavior without requiring labeled fault data.

The ML model runs at the **edge (STM32 microcontroller)** and determines the health of the machine in real time based on vibration and current signals.

---

## 2. Why Machine Learning is Required

Traditional threshold-based monitoring fails to detect early-stage faults because:
- Machine behavior changes gradually
- Fault patterns are not always known in advance
- Manual threshold tuning is unreliable

Machine learning allows the system to:
- Learn normal operating patterns automatically
- Detect subtle deviations
- Adapt to different machines and operating conditions

---

## 3. Learning Approach

- **Type**: Unsupervised Learning  
- **Model**: Autoencoder Neural Network  
- **Training Data**: Healthy (normal) machine operation only  

An autoencoder is well suited for anomaly detection because it can reconstruct normal data accurately while producing high reconstruction error for abnormal data.

---

## 4. Dataset Description

The dataset consists of **time-series sensor readings** collected from:
- Vibration sensor (accelerometer)
- Current sensor (ACS712)

The data is stored in CSV format and represents normal operating conditions during training.

---

## 5. Data Preprocessing

### 5.1 Duplicate Removal
Duplicate samples are removed to prevent the model from overfitting repeated patterns.

### 5.2 Normalization
All sensor values are scaled using **Min-Max normalization**:

`X_scaled = (X − X_min) / (X_max − X_min)`

This ensures stable training and makes the model suitable for embedded deployment.

### 5.3 Reshaping / Windowing
Time-series data is segmented into fixed-length windows so the model can learn temporal behavior.

---

## 6. Autoencoder Architecture

The implemented model is a **fully connected (Dense) autoencoder**.

### Architecture Flow

Input → Encoder → Latent Space → Decoder → Output

- **Encoder** compresses input data
- **Latent space** stores compact representation
- **Decoder** reconstructs original input

The architecture is intentionally kept lightweight to support STM32 deployment.

---

## 7. Model Training

### Training Characteristics
- Trained only on **normal machine data**
- No fault labels required
- Learns the baseline operating behavior

### Loss Function
- **Mean Squared Error (MSE)**

`Loss = mean((Input − Reconstruction)²)`

The loss directly represents reconstruction quality.

---

## 8. Anomaly Detection Method

After training, the autoencoder is used to detect anomalies based on **reconstruction error**.

### Reconstruction Error

Error = MSE(Input, Reconstructed Output)

### Threshold Selection

A statistical threshold is calculated from training errors:

`Threshold = mean(error) + k × standard_deviation(error)`

### Decision Logic

| Reconstruction Error | Machine State |
|---------------------|---------------|
| ≤ Threshold | Healthy |
| > Threshold | Fault / Anomaly |

---

## 9. Model Evaluation

Although the model is unsupervised, evaluation is done using:
- Reconstruction error distribution
- Visualization of original vs reconstructed signals
- Separation between normal and abnormal samples

This confirms that the model generalizes well to unseen data.

---

## 10. Embedded Deployment on STM32

The trained model is optimized for **edge inference**:

- Model weights exported from Python
- Converted to C arrays or TinyML format
- Forward inference executed on STM32
- Reconstruction error calculated on device
- Health decision generated locally

Only minimal computation is required, making it suitable for low-power MCUs.

---

## 11. Advantages of the Proposed ML Model

- No labeled fault data required
- Real-time anomaly detection
- Low memory and computation overhead
- Edge-based intelligence (no cloud dependency)
- Scalable to multiple machines

---

## 12. Limitations

- Requires representative healthy training data
- Sensitive to sensor drift and noise
- Threshold tuning is application-specific

---

## 13. Future Enhancements

- Frequency-domain (FFT) feature extraction
- Sliding window inference
- Multi-sensor data fusion
- LSTM or Conv1D autoencoders
- Adaptive thresholding

---

## 14. Conclusion

The autoencoder-based machine learning model provides an efficient and practical solution for real-time machine health monitoring. Its unsupervised nature, lightweight architecture, and edge deployment capability make it ideal for predictive maintenance in industrial environments.
