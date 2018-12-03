# Rally Car Racing
This is for ECET581 racing project

## Description
### 1. get_cur_pos.py
* This module subscribes the topic `amcl_pose`, `imu_info` and publishs topic `cur_pos` with the frequency of 150Hz
* we need to add the function ```estimate_pos()``` which estimating position based on '**time**' and '**imu_info**'.
##### TODO
* [ ] Working test
* [ ] Complete function `estimate_pos()`
### 2. get_imu_info.py
* This module reads the serial data from IMU and publishs topic `imu_info` with the frequency same with the serial signal.
##### TODO
* [ ] Working test
* [ ] Add Kalman filter
### 3. MyRallyCarCode.py
* This is the main running file which subscribes the topic `cur_pos` and send serial to rally car with the frequency of 150Hz.
##### TODO
* [ ] Working test
* [ ] Adjust parameters