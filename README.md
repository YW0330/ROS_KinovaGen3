# Kinvova Gen3 in ROS
This ros package builds for Kinova Gen3 based on [Kinova Kortex](https://github.com/Kinovarobotics/kortex).

## How to use?
- Clone the project
  ```shell
  $ cd ~/<catkin_ws>/src
  $ git clone https://github.com/YW0330/ROS_KinovaGen3.git
  ```
- Build the project
    ```shell
    $ cd ~/<catkin_ws>
    $ catkin_make
    ```
    若出現編譯失敗的情形，請再次嘗試 `catkin_make`。
- Clean the files 
    ```shell
    $ catkin_make clean
    ```
- Run the program in this package
    ```shell
    $ source ~/<catkin_ws>/devel/setup.bash
    $ rosrun kinova_test <executable file>
    ```
## Reference
- [ros_kotex](https://github.com/Kinovarobotics/ros_kortex)
