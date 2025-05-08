# Autonomous Vehicle Project 

This repository contains the complete source code for an autonomous vehicle project that relies solely on computer vision for navigation and obstacle detection. The system integrates data from GPS, IMU, and camera modules using ROS Noetic, enabling robust and real-time communication between sensor inputs and the vehicle's microcontroller.

## Overview

The project incorporates three key functionalities:

- **Lane & Object Detection:**  
  Utilizes OpenCV to process camera feeds for detecting lanes and obstacles. The core detection algorithms are implemented in the `OBJ.py` file. This detection data is transmitted via ROS Noetic to the microcontroller for further processing.

- **GPS Navigation:**  
  Uses GPS and IMU modules (which provide yaw, pitch, and roll) to continuously update the vehicle's current location and orientation. The vehicle navigates through a sequence of target waypoints provided in the code. Once the final waypoint is reached, the vehicle stops. Refer to the `GPS_NAV.py`, `GPS.py`, and `IMU.py` files for details.

- **Heartbeat Function:**  
  Continuously sends an "I'm alive" message along with the current coordinates to a remote device for monitoring the vehicle's operational status.

## Installation

### Prerequisites

- **Python 3.7+**
- **ROS Noetic:**  
  This is used to integrate sensor data and forward messages to the microcontroller. Please follow the [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation) for details.
- **OpenCV:**  
  Required for real-time image processing.
- **Additional Libraries:**  
  - NumPy  
  - Other ROS-specific dependencies for sensor integration

### Setup Instructions

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/yourusername/Autonomous_Vehicle-main.git
   cd Autonomous_Vehicle-main
   ```
   
2. **Requirements File:**  
   Although a `requirements.txt` file is not provided in this repository, you can create one with the following minimum dependencies:
   ```txt
   opencv-python
   numpy
   # Add any other required Python packages here
   ```
   To install the dependencies (once the file is created), run:
   ```bash
   pip install -r requirements.txt
   ```

3. **Creating Executable Python Scripts:**  
   To ensure that all Python scripts are directly executable on Linux or macOS, add the following shebang line at the very top of each Python file:
   ```python
   #!/usr/bin/env python3
   ```
   Then, make the scripts executable by running the following command in the terminal for each file (for example, `example.py`):
   ```bash
   chmod +x example.py
   ```

4. **Install and Configure ROS Noetic:**
   - Install ROS Noetic by following the [official instructions](http://wiki.ros.org/noetic/Installation) for your operating system.
   - Set up your ROS environment:
     ```bash
     source /opt/ros/noetic/setup.bash
     ```
   - Install any additional ROS packages required for sensor handling and inter-node communications.

5. **Hardware Setup:**
   - Ensure that the GPS and IMU modules are correctly connected and configured.
   - Update the target waypoint list in the GPS navigation scripts (`GPS_NAV.py`, `GPS.py`, `IMU.py`) with the desired coordinates.

## Usage

1. **Launching the System:**
   Start the main application to handle sensor integration and navigation:
   ```bash
   python src/main.py
   ```
   If you prefer using ROS to manage nodes, launch the system with:
   ```bash
   roslaunch your_package your_launch_file.launch
   ```
   
2. **Data Integration via ROS Noetic:**
   - **Lane & Object Detection:**  
     Camera feeds are processed using OpenCV, and detection results are published to ROS topics. This data is then consumed by the microcontroller.
     
   - **GPS Navigation:**  
     GPS and IMU data are continuously monitored and published through ROS topics. The system navigates through sequentially defined waypoints until the final destination is reached.
     
3. **Heartbeat Function:**
   A background process sends periodic "I'm alive" messages along with the current coordinates to a remote monitoring device.

## Contributing

Contributions are welcome! If you have suggestions or improvements, please open an issue or submit a pull request. Ensure that your changes are well-documented and conform to the coding style used in this repository.

## License

This project is licensed under the terms specified in the [LICENSE](./LICENSE) file.

## Contact

For any inquiries or further information, please contact rishabhbhangale@gmail.com or open an issue on the GitHub repository.

---
