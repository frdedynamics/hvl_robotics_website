In this tutorial, you will be given a short introduction on how ROS is structured and how to navigate in it.

## ROS is a Filesystem
ROS2 uses *package* to organize its programs. ROS2 packages are typically organized within a *workspace*, which is a directory that contains all of the resources and code needed for a particular project.

ROS2 packages are typically created using the `colcon build` tool, which is used to manage the dependencies and build process for packages within a workspace.

## ROS Workspace Structure

```
    # my_package structure
    ros2_ws                               # Root Workspace
        └── install                           
        └── build
        └── log
        └── src
            └── my_package                    # Package Name
                ├── action                    # custom ROS Action definitions
                ├── CHANGELOG.rst             # Compliant Changelog
                ├── CMakeLists.txt            # Compilation and Installation steps for C++ Code
                ├── config                    # Configuration files
                ├── CONTRIBUTING              # Contribution Guidelines
                ├── doc                       # Design or other documentation
                ├── include                   # C++ Header files .hpp
                │   └── my_package            # Python Module. Keep the same name of package name
                ├── launch                    # Launch files
                ├── LICENSE                    
                ├── models                    # 3D Models (SDF)
                ├── msg                       # Custom .msg files 
                ├── my_package                # Python modules that can be imported to other ROS packages
                ├── package.xml               # Define the properties of packages and dep
                ├── README.md                 # Package purpose, installation stpe, and Usage
                ├── rviz                      # RVIZ Visualizer files
                ├── scripts                   # Bash scripts 
                ├── setup.py                  # Python Module installation steps
                ├── src                       # C++ Source file .cpp
                ├── srv                       # Custome service file
                ├── test                      # Unit test 
                ├── urdf                      # URDF Files
                └── worlds                    # Gazebo world files
```

So, what you need to know about ROS filesystem so far that:

* You have a folder called **/ros2_ws** in your home directory, and you do everything ROS related in this folder. 
* You place/create all the packages in **~/ros2_ws/src/** folder.
* The folders **/install**, **build** and **/log** are automatically created by `colcon build`.



## Troubleshooting

{: .notice--info}

**What if things gone way too wrong?**
Don't worry. You can always delete the folder with your package name under **/install** and **/build** folders. Then you can run `colcol build` on your terminal (remember to be under the **ros2_ws** directory). CAREFUL! Never delete the files under **/src** folder! This is where your source code is located and no colcon build can bring them back...