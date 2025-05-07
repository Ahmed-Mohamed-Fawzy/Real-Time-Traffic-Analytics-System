# Real-Time-Traffic-Analytics-System
Real-Time Traffic Analytics System
This project implements a Real-Time Traffic Analytics System using MATLAB. It processes video footage from traffic cameras to detect vehicles, count them, and estimate their speeds. The system is designed to handle real-world challenges such as shaky camera footage and varying lighting conditions, making it a robust solution for traffic monitoring.
Features

Vehicle Detection: 

Utilizes spatial domain techniques, including Gaussian smoothing and frame differencing, to accurately identify moving vehicles.


Vibration Removal: 

Employs frequency domain processing with Fast Fourier Transform (FFT) to stabilize footage from shaky cameras, ensuring reliable analysis.


Background Subtraction: 

Implements an adaptive median filtering approach for robust background modeling and foreground extraction, adapting to changing environments.


Vehicle Tracking: 

Tracks detected vehicles across frames to count them and estimate their speeds, providing detailed traffic insights.


Visualization: 

Provides real-time visualization of the processed video with overlays showing detected vehicles, their IDs, estimated speeds, and a cumulative vehicle count.


Demo Mode: 

Includes a simulation mode that generates synthetic traffic frames for testing and demonstration purposes without requiring a video file.



Technologies Used

MATLAB: 

Core programming environment for implementing the system.


Image Processing Toolbox: 

Leveraged for spatial and frequency domain processing techniques.


Computer Vision Techniques: 

Applied for vehicle detection and tracking functionalities.



How to Use

Video Input:

Place your traffic video file in the project directory and name it top_view_cuted_resized.mp4.
Alternatively, modify the videoReader line in the script to point to your video file.


Parameter Adjustment:

Adjust the Region of Interest (ROI) coordinates in the script to focus on the road area in your video.
Tune the minimum and maximum blob area thresholds to accurately detect vehicles based on their size in the video.
Set the position of the counting line where vehicles are counted as they pass.


Running the Script:

Open MATLAB and navigate to the project directory.
Run the script to start processing the video.
The system will generate an output video with annotations and display real-time visualization.


Demo Mode:

If no video file is found, the system automatically switches to demo mode, generating simulated traffic frames for demonstration.



Applications

Traffic Monitoring and Management: 

Real-time vehicle counting and speed estimation for urban planning and traffic control.


Congestion Analysis and Prediction: 

Insights into traffic patterns to mitigate bottlenecks.


Automated Toll Collection Systems: 

Vehicle tracking for toll enforcement.


Research in Traffic Flow Dynamics: 

Data collection for academic and optimization studies.



Notes

The system is optimized for top-view traffic videos but can be adapted for other perspectives with appropriate modifications.
Performance may vary based on video quality, camera stability, and environmental conditions.
For accurate speed estimation, calibrate the distanceCalibrationFactor based on your specific camera setup.

Dependencies

MATLAB: 

Version R20XXx or later recommended.


Image Processing Toolbox: 

Required for image processing functionalities.




Notes
The system is optimized for top-view traffic videos but can be adapted for other perspectives with appropriate modifications.
Performance may vary based on video quality, camera stability, and environmental conditions.
For accurate speed estimation, calibrate the distanceCalibrationFactor based on your specific camera setup.
Dependencies
MATLAB: Version R20XXx or later recommended.
Image Processing Toolbox: Required for image processing functionalities.
