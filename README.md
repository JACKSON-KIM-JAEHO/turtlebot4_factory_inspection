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
###  When adding a new Behavior Tree (BT) node

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

### When adding a new library (e.g., OpenCV, tf2_ros)

    6. **Update `CMakeLists.txt`**

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

    7. **(Optional) Add include paths**
      - Only if the library requires specific include directories:
        ```cmake
        target_include_directories(bt_runner PUBLIC
          ...
          ${OpenCV_INCLUDE_DIRS}
        )
        ```

    8. **Include the library in your source file**
      - Example:
        ```cpp
        #include <opencv2/opencv.hpp>
        ```
### When using the provided helper scripts (`add_bt_node.sh`, `generate_sources.sh`)

    To simplify the BT node integration process, you can use our shell scripts to **automatically generate and register** new Behavior Tree (BT) nodes.   

    1. Run the node creation script

      ```bash
      ./add_bt_node.sh <NodeName>
      ```
      - Automatically create the `.cpp` file in `bt_nodes/` using a Behavior Tree template.
      - Automatically update `sources.cmake` via `generate_sources.sh`.

      2. What is done automatically by the script
      - Add the source file
        - Location: `bt_nodes/<NodeName>.cpp`
        - Template based on `BT::CoroActionNode`
      - Register the new source file in CMake
        - `generate_sources.sh` is invoked to update `sources.cmake`
        - No need to manually edit `sources.cmake`

      3. What you still need to do manually
      -  Add a header file
          -  Location: `include/turtlebot4_factory_inspection/<NodeName>.hpp`
          -  Only needed if your node has complex structure or external usage
      - Register the node in the XML Behavior Tree
        - File: `trees/main_tree.xml` 
        - Add the tag ( Make sure the tag name matches the one used in C++ registration)
          ```bash
          <TakePhoto/>
          ```
      - Register the node in C++
        - File: `bt_runner.cpp`
        - Add the factory registration line:
          ```bash
          factory.registerNodeType<TakePhoto>("TakePhoto");
          ```
      - Add new library dependencies
        - Modify `CMakeLists.txt` as follows:
          - Add to `find_package()`:
            ```bash
            find_package(OpenCV REQUIRED)
            ```
          - Add to `ament_target_dependencies()`:
            ```bash
            ament_target_dependencies(bt_runner
              rclcpp
              behaviortree_cpp_v3
              OpenCV   # New dependency added
            )
            ``` 
          - Add to `target_include_directories()`:
            ``` bash
            target_include_directories(bt_runner PUBLIC
              ...
              ${OpenCV_INCLUDE_DIRS}
            )

            ``` 
