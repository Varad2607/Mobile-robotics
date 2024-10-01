# MuSHR Car Robotics Project

This project explores the implementation of various robotics concepts using [MuSHR cars](https://mushr.io/). The project encompasses a range of techniques in localization, control, planning, and data collection. Below is an overview of the main components:

1. Localization: Implemented using a particle filter to estimate the car's position on the track.
2. Data Collection: Used rosbag for efficient data recording and playback, enabling analysis and fine-tuning of algorithms.
3. Control: Applied two control strategies:
    PID Control: A proportional-integral-derivative controller for simple closed-loop control.
     Model Predictive Control (MPC): A more advanced control strategy that optimizes the car's movements over a prediction horizon.
4. Planning via Waypoints: Implemented waypoint-based planning to guide the car along the provided track.

    
   
This work was a part of EEP 545 Autumn 2023 collaborated with Atharva Pradhan, Soofiyan Atar and Rwik Rana under the guidance of Markus Grotz
