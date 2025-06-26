# 🤖 turtlebot4_factory_inspection
  This repository contains a ROS 2-based autonomous factory inspection system implemented on TurtleBot4 using BehaviorTree.CPP.     
  The robot autonomously follows predefined waypoints, performs angle-aligned photo capturing, avoids obstacles, and returns to its base.     
  Captured images are saved and analyzed to detect changes compared to previous inspections, enabling long-term monitoring of factory environments.       
  The system is developed using a modular C++ behavior tree structure and follows a bottom-up, Agile-style implementation process.

  ---

  ## ⚙️ Development Environment

- Robot Platform: TurtleBot4
- Simulation: Webots
- OS: Ubuntu 22.04 LTS
- ROS Version: ROS 2 Humble
- Programming Language: C++ (BehaviorTree.CPP), Python (Image saving, metadata)


---

## ✨ Features





---

## 🚀 How to Run

```bash
#clone the repository



```
---
## Other_details
####  When adding a new Behavior Tree (BT) node

1. **Add the source file**
   - Create the new BT node `.cpp` file in the `bt_nodes/` directory.
     - Example: `bt_nodes/take_photo.cpp`

2. **(Optional) Add the header file**
   - If the node uses a separate header, place it in `include/turtlebot4_factory_inspection/`.
     - Example: `include/turtlebot4_factory_inspection/take_photo.hpp`

3. **Register the new source file**
   - Open `sources.cmake` and add the new `.cpp` file to the `SOURCE_FILES` list.
     ```cmake
     set(SOURCE_FILES
       ...
       bt_nodes/take_photo.cpp  #New node added
     )
     ```

4. **Register the node in the XML Behavior Tree**
   - Add the new node tag to the BT XML file (e.g., `trees/main_tree.xml`):
     ```xml
     <TakePhoto/>
     ```
   - The tag name must match the one used when registering the node in code.

5. **Register the node in C++**
   - In `bt_runner.cpp`, register the new node with the factory:
     ```cpp
     factory.registerNodeType<TakePhoto>("TakePhoto");
     ```

---

#### When adding a new library (e.g., OpenCV, tf2_ros)

1. **Update `CMakeLists.txt`**

   - Add to `find_package()`:
     ```cmake
     find_package(OpenCV REQUIRED)
     ```

   - Add to `ament_target_dependencies()`:
     ```cmake
     ament_target_dependencies(bt_runner
       rclcpp
       behaviortree_cpp_v3
       OpenCV   #New dependency added
     )
     ```

2. **(Optional) Add include paths**
   - Only if the library requires specific include directories:
     ```cmake
     target_include_directories(bt_runner PUBLIC
       ...
       ${OpenCV_INCLUDE_DIRS}
     )
     ```

3. **Include the library in your source file**
   - Example:
     ```cpp
     #include <opencv2/opencv.hpp>
     ```
