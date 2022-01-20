# ROS2_Tutorial

# Build from source code:
    https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/

# Important  !!!!!!!!!!!!!!!!!!!!!!!!!!:
    . ~/ros2_foxy/install/setup.bash
    . install/setup.bash

# Create workspace:
    https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/#ros2workspace

# Build: 

    All pkg: colcon build
    Only one pkg: colcon build --packages-select <pkg_name>

# ROS2 vs ROS1:

1. Action
2. Launch file written by Python (https://design.ros2.org/articles/roslaunch.html)
3. QoS (https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)
3. ROS 2 DDS/RTPS (https://index.ros.org/doc/ros2/Concepts/DDS-and-ROS-middleware-implementations/)

# Developing a ROS 2 package: 
(https://index.ros.org/doc/ros2/Tutorials/Developing-a-ROS-2-Package/)

1. C++

    ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_cmake

    ament_target_dependencies(<executable-name> [dependencies])

    Config in CMakeLists.txt:
    // Install launch files
    install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    )

    // Install nodes
    install(
    TARGETS [node-names]
    DESTINATION lib/${PROJECT_NAME}
    )

2. Python

    ros2 pkg create <pkg-name> --dependencies [deps] --build-type ament_python

    Config in setup.cfg file:
        [develop]
        script-dir=$base/lib/<package-name>
        [install]
        install-scripts=$base/lib/<package-name>

    Config in setup.py file:
        import os
        from glob import glob
        from setuptools import setup

        package_name = 'my_package'

        setup(
            name=package_name,
            version='0.0.0',
            # Packages to export
            packages=[package_name],
            # Files we want to install, specifically launch files
            data_files=[
                ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                # Include our package.xml file
                (os.path.join('share', package_name), ['package.xml']),
                # Include all launch files.
                (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
            ],
            # This is important as well
            install_requires=['setuptools'],
            zip_safe=True,
            author='ROS 2 Developer',
            author_email='ros2@ros.com',
            maintainer='ROS 2 Developer',
            maintainer_email='ros2@ros.com',
            keywords=['foo', 'bar'],
            classifiers=[
                'Intended Audience :: Developers',
                'License :: TODO',
                'Programming Language :: Python',
                'Topic :: Software Development',
            ],
            description='My awesome package.',
            license='TODO',
            # Like the CMakeLists add_executable macro, you can add your python
            # scripts here.
            entry_points={
                'console_scripts': [
                    'my_script = my_package.my_script:main'
                ],
            },
        )

# Using colcon to build packages: 
    https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/

# Ament documents: 
    https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/

# Writing a ROS 2 launch file: 

    1. C++ 
        # Install launch files.
        install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

    2. Python
        import os
        from glob import glob
        from setuptools import setup

        package_name = 'my_package'

        setup(
            # Other parameters ...
            data_files=[
                # ... Other data files
                # Include all launch files. This is the most important line here!
                (os.path.join('share', package_name), glob('launch/*.launch.py'))
            ]
        )

    3. Written Launch file
        Inside your launch directory, create a new launch file with the .launch.py suffix. For example my_script.launch.py.

        .launch.py is not specifically required as the file suffix for launch files. Another popular option is _launch.py, used in the beginner level launch files tutorial. If you do change the suffix, make sure to adjust the glob() argument in your setup.py file accordingly.

        Your launch file should define the generate_launch_description() which returns a launch.LaunchDescription() to be used by the ros2 launch verb.


        import launch
        import launch.actions
        import launch.substitutions
        import launch_ros.actions


        def generate_launch_description():
            return launch.LaunchDescription([
                launch.actions.DeclareLaunchArgument(
                    'node_prefix',
                    default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
                    description='Prefix for node names'),
                launch_ros.actions.Node(
                    package='demo_nodes_cpp', executable='talker', output='screen',
                    name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
            ])

        Example: 
        https://github.com/ros2/launch
        https://github.com/ros2/launch_ros/blob/master/launch_ros/examples/lifecycle_pub_sub_launch.py

# Passing ROS arguments to nodes via the command-line: 
    https://index.ros.org/doc/ros2/Tutorials/Node-arguments/

# Still not understand: 
    https://index.ros.org/doc/ros2/Tutorials/Composition/

# Something new: Overriding QoS Policies For Recording And Playback
    https://index.ros.org/doc/ros2/Tutorials/Ros2bag/Overriding-QoS-Policies-For-Recording-And-Playback/

# Synchronous vs. asynchronous service clients:  
    https://index.ros.org/doc/ros2/Tutorials/Sync-Vs-Async/
    (Important but I can't understand (sad))

# Something magic in ROS2 (Maybe that is why they call ROS2 is real-time OS)
    https://index.ros.org/doc/ros2/Tutorials/Working-with-multiple-RMW-implementations/

# WTFFFFFFFFFFFF??????????????????
    https://index.ros.org/doc/ros2/Tutorials/catment/#id4

