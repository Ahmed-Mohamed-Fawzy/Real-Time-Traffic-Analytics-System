# Real-Time Traffic Analytics System

This project implements a **Real-Time Traffic Analytics System** using **MATLAB**. It processes video footage from traffic cameras to detect vehicles, count them, and estimate their speeds. The system is designed to handle real-world challenges such as shaky camera footage and varying lighting conditions, making it a robust solution for traffic monitoring.

---

## Features

* **Vehicle Detection**
  Utilizes spatial domain techniques, including Gaussian smoothing and frame differencing, to accurately identify moving vehicles.

* **Vibration Removal**
  Employs frequency domain processing with Fast Fourier Transform (FFT) to stabilize shaky camera footage.

* **Background Subtraction**
  Implements an adaptive median filtering approach for robust background modeling and foreground extraction.

* **Vehicle Tracking**
  Tracks detected vehicles across frames to count them and estimate their speeds.

* **Visualization**
  Real-time visualization with overlays showing detected vehicles, IDs, estimated speeds, and cumulative count.

* **Demo Mode**
  Simulates synthetic traffic frames for demonstration when no video file is available.

---

## Technologies Used

* **MATLAB**
  Core development environment for the entire system.

* **Image Processing Toolbox**
  Used for both spatial and frequency domain techniques.

* **Computer Vision Techniques**
  Applied for robust detection and tracking of vehicles.

---

## How to Use

### 1. Video Input

* Place your traffic video in the project directory and rename it to:
  `top_view_cuted_resized.mp4`
* Or modify the `videoReader` line in the script to reference your video file.

### 2. Parameter Adjustment

* Update the **Region of Interest (ROI)** coordinates in the script.
* Tune the **minimum and maximum blob area** thresholds for accurate vehicle detection.
* Set the position of the **counting line** for vehicle counting.

### 3. Running the Script

* Open MATLAB and navigate to the project directory.
* Run the main script to begin processing.
* Output video with annotations and real-time visualization will be generated.

### 4. Demo Mode

* If no video is found, the system switches to demo mode with synthetic traffic frames.

---

## Applications

* **Traffic Monitoring and Management**
  Real-time vehicle counting and speed estimation for urban planning and control.

* **Congestion Analysis and Prediction**
  Helps mitigate traffic bottlenecks through pattern insights.

* **Automated Toll Collection Systems**
  Enables vehicle tracking for toll enforcement systems.

* **Research in Traffic Flow Dynamics**
  Useful in academic studies and optimization modeling.

---

## Notes

* Optimized for **top-view** traffic videos. Can be adapted for other perspectives.
* Performance may vary depending on:

  * Video quality
  * Camera stability
  * Lighting and environmental conditions
* For accurate speed estimation, **calibrate** the `distanceCalibrationFactor` according to your setup.

---

## Dependencies

* **MATLAB**
  Version R20XXx or later (update with the version you used)

* **Image Processing Toolbox**
  Required for core functionalities.
