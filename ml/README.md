# Machine Learning Module

This folder contains the training and deployment pipeline for the anomaly detection model.

## Model Used
- Autoencoder (Unsupervised Learning)

## Features
- RMS Current
- Vibration RMS
- Frequency Domain Features (optional)

## Training Flow
1. Collect healthy machine data
2. Normalize and extract features
3. Train autoencoder
4. Export weights
5. Deploy to STM32

## Deployment
Weights are converted to C arrays for embedded inference.
