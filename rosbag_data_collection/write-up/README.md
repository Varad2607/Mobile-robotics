While navigating through the basement using rosbag there are considerable differences between 
the path followed by the robot and the path recorded by the rosbag using teleoperation. These
differences could be attributed to:
1. High default velocity causing jerks and requiring frequent stops.
2. Finite delay between sending control signals while teleoperating
3. Noisy real world data.
4. Wheels not fully calibrated at zero position