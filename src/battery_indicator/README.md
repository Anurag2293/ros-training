## Assignment 2 - Autocharging Node

This node monitors the battery status of a robot and automatically manages charging.

### How to Run

1. **Build the package:**
   ```
   cd ~/<workspace>
   catkin build battery_indicator
   ```

2. **Source your workspace:**
   ```
   source devel/setup.bash
   ```

3. **Launch:**

   You can launch either the C++ Implementation (or) Python3 Implementation

   1. ***C++ Implementation:**
   ```
   roslaunch battery_indicator autocharging_cpp.launch
   ```

   2. ***Python Implementation:**
   ```
   roslaunch battery_indicator autocharging.launch
   ```

4. **Verify Autocharging Working:**

   - In new terminal, source and subscribe the `/battery_status` topic
   ```
   source devel/setup.bash
   rostopic echo /battery_status
   ```

   - In another terminal, source and subscribe the `/error_status` toipc
   ```
   source devel/setup.bash
   rostopic echo /error_status
   ```