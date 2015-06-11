^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys_gazebo_general
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.9 (2015-06-11)
------------------
* Replace shared_dynamic_cast to dynamic_pointer_cast because shared_dynamic_cast is deprecated from boost 1.53
* [hrpsys_gazebo_general] Use find_package macro to look up collada_jsk_patch package
* [hrpsys_gazebo_general] Fix typo
* [hrpsys_gazebo_general] Fix path for catkin build
* add use_joint_effort for using effort on velocity feedback mode
* Contributors: Ryohei Ueda, YoheiKakiuchi, Iori Kumagai

0.1.8 (2015-01-09)
------------------
* fix CHANGELOG order, https://github.com/ros/rosdistro/pull/6794

0.1.7 (2015-01-09)
------------------
* add pthread for link only for raling 64bit
* Contributors: Kei Okada

0.1.6 (2015-01-08)
------------------
* update CHANGELOG.rst
* add dl to target_link_libraries
* add SPAWN_MODEL and fix for PAUSE
* change model name when spawning robot model
* update setup.sh
* fix using SPAWN_MODEL argument
* udpate PubQueue.h for matching gazebo_ros_pkgs
* Contributors: Kei Okada, YoheiKakiuchi

0.1.4 (2014-10-23)
------------------
* install compile-robot-model-for-gazebo.cmake
* deletece unnecessary file. added xacro file

0.1.3 (2014-10-12)
------------------
* set CMAKE_BUILD_TYPE to install, also support RelWithDebInfo for deb release
* install files
* add compile_robot_model_for_gazebo.cmake and generate model and launch file for samplerobot simulation.
* Contributors: Kei Okada, Masaki Murooka

0.1.2 (2014-10-06)
------------------
* add gazebo_ros, gazebo_msgs, and gazebo_plugins to dependency of hrpsys_gazebo_general, eusgaebo.
* Merge pull request #111 from k-okada/hrpEC_with_hrpIo_gazebo
  compile hrpEC liked with hrpIo_gazebo
* add COMPILE_DEFINITIONS for hrpEC
* Contributors: Kei Okada, Yohei Kakiuchi, Masaki Murooka

0.1.1 (2014-09-26)
------------------
* Initial release of hrpsys_gazebo_general
* Contributors: Kei Okada, Ryohei Ueda, YoheiKakiuchi, Masaki Murooka, Shinichiro Noda
