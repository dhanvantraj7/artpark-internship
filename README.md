# IMU Sensor Fusion Implementation

## Project Overview
Implementation of sensor fusion algorithms for IMU data processing, focusing on Kalman and Extended Kalman Filtering (EKF) for accurate motion tracking.

## Technologies Used
- STM32 (NucleoF446re, STM32F103C8, NucleoG070)
- IMU Sensors:
  - ISM330DLC
  - STEVALMKI184V1
  - STEVALMKI185V1
- GPS Module: SparkFun GPS Breakout - NEO-M9N
- Development Environments:
  - STM32CubeIDE
  - MATLAB
  - Arduino IDE
  - Visual Studio Code
  - Go Lang

## Key Features
- IMU sensor data acquisition and processing
- Implementation of Kalman Filter in multiple languages (C, MATLAB, Go)
- Extended Kalman Filter (EKF) implementation
- Real-time data visualization
- GPS integration
- Adaptive filtering techniques

## Project Structure
- `/src/kalman` - Kalman filter implementations
- `/src/ekf` - Extended Kalman Filter code
- `/matlab` - MATLAB scripts for algorithm verification
- `/visualization` - Data visualization tools
- `/docs` - Documentation and mathematical models

## Installation & Setup
1. Clone the repository
2. Install required dependencies
3. Configure STM32CubeIDE for hardware integration
4. Follow sensor-specific setup instructions in /docs

## Usage
- Sensor data collection scripts
- Filter implementation examples
- Visualization tools
- Performance analysis utilities

## Algorithm Implementations
- Basic Kalman Filter
- Extended Kalman Filter
- Adaptive Kalman Filter
- Particle Filter

## Future Work
- Motion tracking optimization
- Enhanced noise parameter estimation
- Real-time performance improvements
- Integration with additional sensor types

## License
[Add License Information]

## Contributors
- Dhanvant Raj
