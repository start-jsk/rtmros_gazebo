^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys_gazebo_atlas
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.9 (2015-06-11)
------------------
* [hrpsys_gazebo_atlas] Do not compile iob if no atlas_description is available
* [hrpsys_gazebo_atlas] Fix path for catkin build
* add measured polaris points
* Contributors: Ryohei Ueda, YoheiKakiuchi

0.1.8 (2015-01-09 12:27)
------------------------

0.1.7 (2015-01-09 02:30)
------------------------

0.1.6 (2015-01-08)
------------------
* update polaris
* fix polaris
* Do not generate atlas model if no atlas_description is available
* Contributors: Ryohei Ueda, YoheiKakiuchi

0.1.5 (2014-11-04)
------------------

0.1.4 (2014-10-23)
------------------

0.1.3 (2014-10-12)
------------------

0.1.2 (2014-10-06)
------------------
* add polais xp900 euslisp model
* Contributors: YoheiKakiuchi

0.1.1 (2014-09-26)
------------------
* fixed atlas v0 endcoords
* fix real2model
* include atlas_bringup.launch in launch file for hook hand atlas
* (catkin.cmake,CMakeLists.txt) : Remove deprecated abc_end_effectors and fix end-effector name (without colon) according to https://github.com/fkanehiro/hrpsys-base/pull/301
* fix collision mesh of hook hand model
* added models files for hook hand
* fixed parameter for walking
* fix indent
* set default value of impedance-controller-param
* added atlas_hrpsys_dashboard script and call it.
* added sandia-hand-command-publiser.l
* deleted unnecessary lines and enable dashboard
* remove unused variables
* add force-offset file and load that file
* modify atlas-interface.l for deleted files
* move euslisp files of touchit
* move euslisp files of atlas-manipulation
* delete unnecessary fiels
* move euslisp files of ladder
* move euslisp files of utilities
* move euslisp files of atlas-multi-locomotion
* move euslisp files of atlas-joint-state-compresser
* move euslisp files of atlas-vehicle-motion
* fix stabilizer param
* fix iob for atlas_v0
* move atlas setting functions from atlas-**-interface.l to atlas-interface.l
* modified iob.cpp to support atlas_v0 and atlas_v3
* added euslisp and launch file for atlas_v0 and atlas_v3
* added load-ros-manifest for sandia_hand_msgs and atlas_msgs
* Merge pull request #67 from mmurooka/add_end_effectors_to_hrpsys_conf
  add end_effectors to conf file
* added comment about joint order in iob.cpp
* fix joint index order of atlas in iob.cpp
* add end_effectors to conf file
* (hrpsys_gazebo_atlas/launch/atlas_hrpsys_ros_bridge.launchw) set USE_WALKING true, USE_IMPEDANCE_CONTROLLER true, USE_SOFTERRORLIMIT false
* (hrpsys_gazebo_atlas/catkin.cmake) fix for when hrpsys is installed
* (hrpsys_gazebo_atlas) add message generation
* (iob/CMakeLists.txt) use hrpIo_atlas
* (atlas_client.py) rename AbsoluteForceSensor -> RemoveForceSensorLinkOffset
* set (launch/atlas_hrpsys_bringup.launch, launch/atlas_hrpsys_ros_bridge.launch) USE_COLLISIONCHECK false
* (hrpsys_gazebo_atlas) create RobotHardware.so and libhrpIo_atlas.so
* added hrpsys_gazebo_atlas/REAME.md: instruction for launching
* deal with both of catkin and rosbuld in atlas_client.py
* not call resetJointGroup.py when servo on.
* fixed atlas_client.py for new hrpsys configurator
* compile_collada_model now create dependency tree automatically, so we don't need to make dependency in user-space cmake
* remove wstool install laser_assembler and check if the directory exists when compile atlas_laser_snapshotter
* remove hoge/fuga for travis
* describe comment
* adding atlas_description
* remove atlas_description from build dependency
* add interface to move real robot
* add rtmros_tutorials
* modify ik controller in order to use other robot
* add laser_assembler, but it does not catkinzed in groovy so no effects
* add baxtere_description for baxtereus
* update end-effector definition for atlas
* return true or false from initialize_for_add_sensor
* use function to initialize in main function
* find articulated_system with _motion ;; for example, robot0_motion, ....
* find kmodel id instead of using the string 'kmodel0'
* set attribute according to sid of link instead of using name of link
* changed scripted posture for first door
* changed foot placement to widen legs in atlas_door_motion_player.lanch
* bug fix: tf-end-coords function transformation fix
* changed foot placement in atlas_door_motion_player.lanch
* display message when move arm was changed
* adding padkontrol
* change origin-key to :rarm and :larm atlas-ik-controller.l
* update image_view name
* removed output log comments in touchit.
* add subscriber to change ankle pitch joint angle
* simplify atlas-nlopt-ik-test.l, it is better for single arm ik problem?
* small :dif-rot-ration is better ?
* fix ik threshold bigger than ik mvoe distance
* fix threshold for ik rotation, and add some log
* target-coords attitude <- end-coords one, and publish im to rviz
* display message when menu is selected
* add tf-end-coords function for update target-coords in condition that rotation-axis = nil
* add tf-end-coords function for update target-coords in condition that rotation-axis = nil
* bug fixed in the case of not making viewr window in touchit
* add publisher to toggle ik mode in padkontrol
* not make viewer in touchit-server.
* set use torso nil in ik-controller by default
* set use torso t in ik-controller by default
* set rotation axis nil in ik-controller by default
* deleted debug lines and enable to set negative value for argument.
* changed range of touchit threshold : [0:100] -> [-1:100]
* chnaged minimal thre -1 in order to enable cancel. If you set negative thre, touchit is cancelled immediately.
* changed default value of touch-it-control-proc arguments: axis = nil and ik-type = :arm
* not set ik-type and axis when calling touch-it-control-proc in order to use default value.
* changed endcoords for hookhand.
* added touchit_thre.sh for changing threshold force value of touchit
* publish touchit threshold with touchpad
* fix end-coords of hook to end point
* add hook end-coords and set defalt
* add atlas_pcl_divider.launch
* set threshold with arguments in touch-it-util
* set threshold by topic in touch-it
* set origin-key to free
* using grasp frame_id
* enable to set devided number of touch it by argument
* changed devied number depending distance between current and dest
* set rotation axis nil when center sphere was moved
* changed scale and size in publish-touchit-result
* added option set-user-pinned for teleport-to
* added gazebo_atlas_door.launch drc_practice_hook_atlas.launch for door environment simulation
* set touchit information to the global variable and send angle-vector smoothly
* send data as feedback from  global variable, and set status depending on finished result.
* added gazebo utility scripts
* add atlas-door-cheat.l for teleportation
* diable xterm
* adding padkontrol
* fixing padkontrol
* adding padkontrol
* adding fc_gazebo.launch
* move slowly
* fix angle-vector bow -> bye
* add atlas-motion-sequence-player.l
* revert to r6616
* set end-coords of ik-server
* enable to set end-coords from client
* set executable atlas-motion-sequence-player.l
* update parameters
* changed position of footplace marker
* add orbit function
* add centroid-offset value for toe balance
* add joint limitation 11 deg for crotch-r joint
* update footstep parameters
* move roi-reconfigure-call.l
* reconfigure relay for ROI
* add hand roi_viewer
* added ik parameter for not using null space, which is now comment out.
* changed log from warning-message to ros-warn
* adding some steps
* remove pre-call model2real for safety
* climb ladder using pull force
* comment out dummy-ri and wait 10 sec before making *ri*
* fix minor bug
* added ros-warn log in touch-it.l
* add force sensor tf
* add foot sensors subscribe
* pull force constraints add
* inital commit atlas-ladder-dynamic.l
* add foot step parameter
* update color of wrench_string
* add respawn=true to rotate nodes
* inital commit atlas-motion-sequence-player
* lower freshrate
* add atlas_wrench_string_publiser
* add atlas-wrench-string-publisher
* decrese the freshrate
* set nan in joint state compressed by default and dont publish nan joint
* fixing name
* sleep before die
* adding script
* adding toggle_mux script and fix name
* adding mux to lhand and rhand
* use default robot description
* add sample
* set output screen for touchit
* changed to use ros-warn
* add sensor tf
* add door foot in launch file
* remove :cancel-all-goal of call-touch-it-server
* fix hand rotate 120 -> 0
* add rotate fisheye
* add tf for force sensor
* fix bugs when joint-names include hand information
* add rotate value for look-hand functoin
* fix choosing argument GAZEBO
* add argument for choosing GAZEBO
* update
* move image_gui
* change image_rect -> image_rect_color
* use interactive joint by default
* add second door open motion, switch *door-id* variable
* add open-second-door-front in atlas-door.l
* add atlas_door_motino_plauer.launch
* chmod a+x 2
* chmod a+x atlas-door-motion-plauer.l
* fix bug of sandia hand name
* add sandia-hand joint state publish
* remove unused joint gain
* spacenav can be used when ik-stop mode
* update to use rotated image
* fix rotation-axis from goal_id -> seq
* new atlas-door-motion-player add,
* head_snap viewer image_rect -> image_rect_color
* add touch-it server
* added open-first-door-front
* add nth-angle-vector function for setting function to angle-vector-list-list
* adding other joints
* adding script to align windows
* remove preview function for touch-it
* changed scripted pose in atlas-door.l
* not display info of multisense-sl-compresser
* adding pointcloud from hands
* change image_rect -> image_rect_color on head_camera
* shut you face
* add move max switching with the result of inverse-kinematiacs
* divide /atlas/joint_state_compressed to /atlas/~ and /multisense_sl/~
* add multisense_sl joint state callback
* change topic name of multisense_sl from /atlas/~ to /multisense_sl/~
* look at callback add
* update
* added look-hand function
* bug fix for arm only inverse-kinematics
* add sample code
* add pre-manip-pose
* set rotation-axis for call-touch-it-server function
* add joint-state-subscriber2 for joint feedback from rviz
* add pose for door
* remove :draw-objects function in loop of joint-state-subscriber
* remove look-at function
* add atlas-drill-motion-paler.l for drill motion plau in angle-vector-player.lk
* changed standing point for opening door
* initial commit angle-vector-player.l, please use with eus_gui.py
* update the parameter of rate
* adding topic_buffer to force sensor
* using new parameter
* adding topic_buffer
* not display output of topic_buffer_server
* adding eus_gui
* adding eus_gui
* adding eus gui
* add all_viewer to ocs.launch
* using ROS namespace
* remove DEV specification
* not use index when compress joint state
* fixing value of delay to be displayed
* add timer for debug
* using timer
* update
* update actionlib
* using parameter
* use JointState instead of JointStateCompressed
* remove unused functions of atlas-joint-interface-fc/l
* some bug fix, jsk_interactive/atlas-joint,l -> atlas-ik-controller.l
* add marker-menu-callback2 for robot-pose reset and stand
* add atlas interface in fc to move robot
* added scripted motion for opening door.
* update for using topic
* adding parameter for topic_buffer_client to run in topic mode
* add touchit-target values for touchit ik mode change
* enable to set axis for touch it server
* adding atlas ping gui
* adding ping gui
* remove stderr output
* send server :set-lost has bug of undefined variable
* add publish-touch-result when call-touch-it-server
* add global variable touchit-reach and touch
* gui for rosping
* adding rosping plugin
* adding gui for rosping
* update parameters
* update parameter
* add call-touch-it-server function
* add dummy *ci* and dummy real2model for local touch-it-server
* update using image
* move buffer_server to fc
* add lifetime to touchit result marker
* publish touchit result text marker
* make real robot interface in touch it server
* changed window tile and color depending on topic name
* changed node name in atlas_touchit_server_ocs.launch
* add roi image
* update parameters
* set topic name with environment variable in touchit_server launch files
* read environment variable for topic name
* changed indent in touch-it-util.l
* add snapshot gui
* add testing viewer for atlas
* add comment setting
* fix parameters
* some bug fix hogehoge
* improved touch it
* add touchit callback
* adding images
* update image_transport
* adding image topics
* more beautifully
* add solve-triangel functions and some bug gix
* add bound chekc for grobal variables
* update image rotate
* adding color
* adding subgraph
* add fisheye to image_transport
* rename the file
* adding pointcloud
* add joint state topic graph
* add icons
* fix path to resetJointGroup.py
* clearn parameters
* set debug-view nil, and added try-door-demo function
* fix variable names
* solve inverse kinematics in the new configuration for turning valve, use HKU coordinate
* add resetJointGroup to servo_on/off
* print collsition log to terminal only when the collision occured
* add publish-eus-obj function for triangle and foot-step display
* remove unused function and waist-fix
* added functions for opening door
* befrore call :old-reset-manip-pose, check
* added atlas-door.l
* fix end-coords because reset-manip-pose are changed
* fix joint state subscriber, joint staes have joint angles and names
* using raw pointcloud instead of filtered pointcloud, self_filter is not stable
* using raw pointcloud
* using old values
* fixing topic name
* using raw pointcloud
* forget to remap update??
* fixing namespace
* rotate 120 deg right hand and drill motion
* instantiate robot from atlas_client.py
* move script/hrpsys_cofnig.py to src/hrsys_gazebo_atlas/atlas_client.py
* adding pcl roi stuff
* fixing pcl roi stuff
* pcl concatenater fixing topic name
* don't display info
* don't use script to update topic_buffer_server
* remove un-used diagnostics data
* adding PCL configuration for ROI
* not use robot_description_ocs
* create ATLASHrpsysConfigurator to resetJointGroup
* create resetJointGroup()
* add controller setting for using limb trajectory from lisp interface
* set default origin-key -> :rarm
* set robot_description param in atlas_send_tf_ocs
* remap tf to tf_ocf in ocs
* set atlas-torso limit 40 -> 10
* start-ik-server -> start-ik-server and loop-ik-server functions
* use default robot_description in ocs
* add torso3 inverse-kinematiacs, only use torso-y
* inital end-coords cheange to id=1
* add torso2 mode for ik-server, just move x y z yaw joints of pelvis
* add *ik-stop* variable for ik-controller, default value = 0
* bug fix, defautlt ned-coords, if=0
* use atlas-end-coords.l in spite of set-end-coords function
* intial commit atlas-end-coords.l, switch some lim :end-coords
* delete use_interactive_endcooreds arg in fc.launch
* comment out ik-controller
* 3d mouse joint angle -> rviz
* remap tf topic name used by ik-server
* rotate hand image to map coords
* update foot convex every time inverse-kinematics was called
* set target to correct position when frame changed
* bug fix: centroid objects geenrate twice
* recreate foot-convex may add some error of ik
* fix foot-convex when robot posture move far away from now state
* delete rviz for endcoords interactive marker
* use joint state publisher for joint interactive marker
* remove unused comment functions
* add realmodel to model function
* fix coordinate transformation, when pelvis rotation, before version wont be move
* fix target-coords of ik-server from pelvis
* add respawn for ik-server
* enable dual-arm-ik, but not good
* transformation fix in local world coordinate
* change node name of im-marker
* ik-server enable to set constrains parameter, for now, parameter will be sent with s-string
* fix some cooridnates bugs
* add method to publish joint-states
* fix interactive marker pose
* bug fix, mouse-mode check before mouse-mode update
* add *real-robot* objects for real angle-vector update
* skip 3d mouse, whne mouse-mode nil
* add main-loop function demo-pos-conttoller2
* update parameters
* fix bag and indent
* fix and add some variable names
* fix global variable name  -> **
* add function to set marker position
* add function to get tf from map to robot
* add method to set origin
* publish arrow marker to see origin and target
* add some message for interactive marker connection
* adding pcl concatenater
* fixing params
* add interactive mareker callback
* fix some parameter like move step on dmeo-pos-controller
* add fix-limb-coords valiable for error summatino
* concatenate pointclouds
* deom-pos-controller fix, coordination fix
* add atals-eus-ik.l node
* add script to all rtm/ros programs
* update reset manip pose
* add arrow object for target-coords visualize
* large window irtviewer
* do not launch hrpsys_dashboard
* forge tto add USE_DIAGNOSTICS
* disable pose button
* mv obsolated launch files to old.launch
* add diagnostics for atlas_hrpsys
* adding new image
* adding image_transport
* add checking existing force
* update joystick device file
* to reduce sumation of error, solve inverse-kinematics for both legs after ik-service-call
* remove USE_CONTROLLER arg
* fix typo
* remove node
* update parameters
* renaming file
* removing file
* updating stuff
* remove unused functions and some bug fix about function references
* rename file
* rename fileatlas_ik.launch
* adding two launch file for narrowband
* solve inverse kinematics comunicating with ik-server
* divide tf-related launch file
* delete specific_transform_publisher in CMakeLists
* adding triangle gui
* adding triangle gui
* fix bag : send marker tf to ocs
* send marker tf to ocs
* add servo on/off scripts
* successufully call ik-server and get angle-vector, look like correct
* fix for head-less mode
* set OUTPUT to screen
* bug fix, quotanion caluculation fix
* segmentatino fault fix when non normalized quotanion detected
* fix typo and use use_cache, instaed of ~use_cache
* fix topic name - -> _
* move specific_transform_publisher and subscriber to jsk_topic_tools
* any option for inverse-kinematics can be used
* fix pelvis coords as foot coords to orgin, because fullbody ik is supported and the center of gravity is no the support plane
* add start hrpsys_atlas_dashboard
* use atlas_hrpsys.launch
* do not subscribe diagnostics/rosout, send go_actual for all mode buttons
* add rh.q to logger
* publish joint state compressed to move real robot
* publish joint state to visualize the result of ik
* initial commit atlas-ik-controller.l, for now, just the same as atlas-dual-arm-ik.l
* add filtered force sensor
* set additional gain for shoulder joints, but ik fail with strange points
* remove unused functions and comment, normal-ik -> normal-ik-with-collisoin
* publish tf from map to pelvis when using gazebo
* change for using topic_buffer
* add hrpsys_atlas_dashboard
* add parameter for using hrpsys-simulation with atlas
* add using roi image in multisense_sl
* single arm drill manipulation, base link only move z-directions
* don't use cache when using dynamic_tf_publisher
* remove inverse kinematics for coordinates settings
* add header file for specific_transform_subscriber
* atlas-dual-arm-ik depends on atlas-fullbody-ik.l
* use dyanmic tf publisher in specific_transform_subscriber
* use dynamic_tf_publisher launched in ocs
* divide low bandwidth launch file into two
* adding comment
* supporting preempt
* implementing using touch-it-control-proc
* loop -> touch-it-control, iterative method -> touch-it-control-proc
* rename touch-it-control to touch-it-control-proc
* indent
* untabify
* untabify
* untabify
* read initial force as offset in touch-it-util.l
* updating sensor frames
* fix the orientation and trnaslation of force sensor on the arms
* added atlas_touchit_server.launch
* added touch-flag and overwriting stamp of posestampedin touchit.
* improved touchit loop process
* add simple-rsd-play function, for animation, and send commnad to robot
* remove upper point cloud before dividing
* add demo-hand-climb-ladder function, climbing ladder motion with hans supports
* move some functions from touch-it-server.l to touch-it-util.l
* read env and set topic name
* disable fulutaractive markers in default
* add spline interpolation, for now, it is related to euslib/demo files
* fixed typo in touch-it-server.l
* added touch-it-controll function for using without actionlib
* added guard of recalling setup-end-coords
* changed interpolation time in atlas-impedance-calib.l
* added touch-it-util.l
* fix a lot of stuff
* good bye robot_description
* remove robot_description
* miracle static walk for climbing ladder
* added touch-it-server.l touch-it-client.l
* do not start ik server loop if *do-not-start-ik-server* is defined.
* add  -hold option to keep window after exit
* add USE_CONTROLLER to hrpsys_atlas.launch and update Makefile.hrpsys-base to create icon
* fix to use atlas_v3 model for hrpsys(non-gazebo) simulation
* some parameter turning, especially, ladder height 30cm -> 30.5cm
* add demo function, climb ladder animation
* can solve, but with collision
* added option for joints version and endcoords version of interactive marker
* added option for interactive marker
* include ik_server and set endcoords interactive marker default false
* add some functions for static climbing
* added atlas_hrpsys_real.launch file for setting
* make image smaller and rate high.
* do not consume alot of cpu
* changed backgroud color of roseus window.
* enable to select whether make viewr or not in atlas-init-ex
* change launch file of interactive_marker
* add output_frame in divided pointcloud
* load atlas-impedance-calib.l in atlas-interface.l
* added atlas-impedance-calib.l
* add hand interactive marker
* include msg compresser in atlas_hrpsys.launch
* add msg compresser for low bandwidth
* add compresser for joint_states of multisense_sl
* put together joint states of body and hand
* added drc-valve.l
* check self collision in torso ik in the ik server.
* fixed to use foot-convex of robot posture.
* added option arguments for fullbody-ik-main
* add robot_state_publisher for compressed
* add msg of compressed joint state
* add compresser and decompresser of joint states
* added if for interactive joint marker and rviz in atlas_hrpsys.launch
* adding rviz config
* updating coloring
* add sample to visualize divided point cloud
* changed ik target coords to be far from robot in x direction because target is too near and self collision occurs.
* generate pcl caller scripts
* use fullbody-ik-with-collision instead of fullbody-ik in ik-server.
* added some arguments such as thre, rthre, collision? in ik-main
* add topic_buffer_server in atlas_pcl_divider
* add atlas-fullbody-ik.l, it is mostly copy of atls-dual-arm-ik.l, so I should edit atls-dual-arm-ik.l to generalize and use it.
* add sample program to display pointcloud
* add script to generate atlas_pcl_divider.launch
* update weight
* update to use atlas_v3
* include mjpeg_server in atlas_imagetransport
* add the comment for instruction in atlas-dual-arm-ik.l
* added atlas-valve.l
* comment out reset-pose in atlas-calibration-pose
* added init-for-drill-grasp for teleporte in drcsim
* changed interpolation time in atlas-hrpsys-test.l
* update sensor parameters
* fix triangle parameter, 30x15 cm right triangle
* update end coords offset
* adding launch file to launch ik server
* updating for catkin
* updating to take balancing into account
* update ik server
* added atlas-ladder.l and drc_ladder.l
* adding output=screen
* updating to support arm, torso and fullbody ik and joint state
* add atlas-eus-ik-sample.launch, launch atlas-eus-ik server and clinet
* add atals-eus-ik-client.l, somethings strangee
* update end coords
* use quickhull function, and remove gen-foot-convex
* add additional-wieght-list parameter to atlas-eus-ik.l
* update
* use iob.h under /lib/io
* add my-object function, display robot cog triangel convex and drill
* add gen-foot-convex methods, generate convex hull of foot, for now, this can be user for only atlas
* add use-messages and period option in imagetransport
* single arm ik version commit, when solving ik-main, set target-limb '(:rarm :rleg :lleg)
* some parameter tune
* set include_directory(hrpsys/inlcude) before /opt/ros/DISTRO, use user package before system package, fixed for hrpsys 315.0.0
* rename resized_imagetransport -> resied_image_transport, if you have problem, please svn up under jsk_visioncommon
* update endcoords
* add read_digital_output for hrpsys 315.0.0
* update end-coords
* update end-coords
* reduce crotch-r joint limit -> +-5
* model fat and collision ik-revert support
* add collision check to ik-nmain
* fix init-grasp-pose for collision avoidance
* atlas-dual-arm-ik.l with new atlas_V3.l, please remove models/atlas_v3.l && make
* add gen-yaml-rotation function, to generate models/atlas_v3.yaml, end-coords fix
* changed end-coords config in atlas.yaml and atlas_v3.yaml and remove overwriting of reset-manip-pose in atlas.l
* update atlas_octomap.launch
* add atlas_scan_to_cloud_long_range.yaml
* fix typo
* adding torso
* inital commit keyboard-coords-fix.lk
* adding atlas-eus-ik
* removing gensrv
* removing srv
* add gazebo feedback and solve ik
* include atlas_joint_marker in atlas_hrpsys.launch
* bug fix: leg-coords-fix-from-real function
* add leg-coords-fix-from-real function, feed-back leg coordf from gazebvo
* reach ground and return to original pose.
* set option for atlas_web.launch and set false default.
* add triagle model, just load, and get *triangle*
* add demo-function for simulation play
* adding service for EusIK
* adding circle to image_view2
* adding www directory
* verbosing messages
* adding output=screen
* adding atlas_web.launch
* fix dt for atlas_v3
* reverted last commit of atlas-interface.l. set with-hand nil in (atlas-init) by default.
* demo-pose-controller max evaluation 30 -> 1000
* send angle to real robot when *ri* exits
* 3d mouse mode add, when you click 3dmouse buttton, 3d mouse mode will start
* adding mode line
* launch file to start atlas triangle ui
* adding atlas_web.launch, web UI
* reverted last commit of atlas_laser.launch
* added (init-for-drill) for drill task
* added drc_practice_task_6_with_ground_plane.world
* added drc_practice_task_6_with_ground_plane.launch
* tmp commit, unstable atlas-dual-arm-controller.l
* fix topic name
* implmeneted ros connection
* adding a script to publish triangle points
* subscribe triagnlepoints
* adding triangle point
* irt movable hogehoge
* some para tune
* fullbody-ik overwrite for getting failure value
* adding another plane detector to detect wall
* change the value of filter to see the near area
* fix centroid constraints
* check if gazebo before find_package
* hrpsys_gazebo_atlas only support groovy
* do not generate model when collada_urdf_jsk_patch is not found
* add depends to collada_urdf_jsk_patch
* defualt set-user-mode nil
* do not use rosrun in catkin.xmake
* depends to collada_urdf_jsk_patch
* initial commit atlas-dual-arm-controller.l, with spacenav, solve ik
* added instruction comment for test in atlas-moveit.l
* update making robot
* adding a program to detect planes
* adding sandia_hand_teleop
* update drcsim launch
* updating max_range parameter
* adding a launch file to launch sensor stuff
* fix for source compile
* add model compile code
* add link_directories
* fix message
* set USE_VIEW as default true
* update topic name
* update topic name
* use SVN_DIR to set source directory
* catkinize hrpsys_gazebo_atlas
* changed package name atlas_utils -> drcsim_gazebo
* add dot.rosinstall
* remove hand_controller and bdi_action
* add atlas_hrpsys_drcsim.launch
* update parameter
* initial commit for describing atlas laser pipeline
* update
* update laser pipeline
* update frames
* update parameters
* minor changes in atlas-hrpsys-test
* added collision_pair config in CMakeList.txt
* add dummy pointcloud publisher and mux to select them
* added time argument to model2real-safe
* added reaching hand to the groundfuntion
* update pose
* add atlas-pose
* bug fixed, and removed line of rosbag
* add start node for laser
* change hrpsys_rate -> 333
* add intensity filter
* update filter parameter
* added record_rosbag lines to atlas_hrpsys.launch, default is off.
* added rosbag_record_atlas.sh
* delete co, and input current angle, to use collisoin detector just for checking in hprsys_conf.py
* return to reset-manip-pose in test-auto-balancer-balance
* check if the joint_states are published in iob.cpp
* add self filter node
* add self filter setting
* change dt to 3ms at atlas
* added atlas-hrpsys-test.l
* fix joint_trajectory_controller -> follow_joint_trajectory
* add joint group controller setting
* added (atlas-balancing-demo) in atlas-hrpsys-demo.l
* added some test functions for hrpsys test
* change interporation time 1500 -> 3000 in (model2real)
* added test functions of hrpsys
* added draw-real-robot and model2real-safe.
* update hand model
* add hrpsys_dashboard to atlas_hrpsys.launch
* add atlas_hrpsys_loopback.launch simple loopback mode for hrpsys
* update link names
* change: default launching trajectory controllers for each limb
* fix typo
* added comment line for printing publishing topic.
* change not publishing command while servo off
* set ankle kp_velocity 0 in iob; if ankle kp_velocity > 0, atlas blows.
* add code for treating servo state
* fix: update to r5733
* added write_digital_output_with_mask to iob.cpp in hrpsys_gazebo_atlas
* minor update
* add sample to atlas-moveit.l
* do not use compile message at geometry_msgs
* update simple car model
* add atlas-moveit.l
* add trajectory controller configuration
* set kp_velocity 50 in all joints except for ankle joijnts
* changed leg gain value in (my-init)
* added gains to PDgain.sav (v3 has two more fixed joints than old atlas)
* fix end-coords coordinate when generate eusmodel. fixed configuration in atlas.yaml and atlas_v3.yaml.
* added (atlas-init-ex) in atlas-util.l : the same function with (my-init)
* fixed neck joint name
* added argument error check to (atlas-set-servo-gain-by-torque-limit)
* changed load atlas model file atlas.l -> atlas_v3.l
* chnaged default ROBOT_NAME atlas -> atlas_v3
* removed uncomment unnecessary lines in iob.cpp
* edited iob.cpp for atlas_v3 : change kp_velocity 100 -> 0, fixed joint_id_real2model array, changed.
* update iob.cpp for v3
* update atlas-set-servo-gain-torque-limit for v3
* convert atlas urdf (not v3 model) in atlas_description; [[ not compatible with old drcsim ]]
* comment out some packages at groovy and latest drcsim
* yaml file for atlas_v3
* add compiling atlas_v3
* added iob function: write_command_torque and read_actual_velocity
* add atlas setting for end_effectors
* bug fix: missing link
* inital commit altals^nlopt-ik-test.l
* added walk utility function: start and stop walking keeping autobalancer
* added function for qual door task
* bug fixed in atlas-hrpsys-demo.l
* added atlas-hrpsys-demo.l atlas-manip-obj.l
* add drc simple vehicle
* fix compiling for fuerte
* add hrpsys-ros-bridge test launch for atlas
* add atlas_hrpsys_ros_bridge and use it from atlas_hrpsys_bringup and atlas_hrpsys_simulation
* add test for atlas-hrpsys-ros-bridge-test
* rename hrpsys -> hrpsys_tools
* fixed some bugs in hrpsys_gazebo_atlas/euslisp/
* modified README
* added README for moving atlas with euslisp
* add configuration for sequencer groups and modify launch for using it
* fixed bug of hrpsys script and launch in hrpsys_gazebo_atlas
* fix package name hrpsys_gazebo -> hrpsys_gazebo_atlas
* fix package name
* fix package name
* mv hrpsys_gazebo_atlas/jenkins/ to hrpsys_gazebo_general/
* re-organize rtmros_common, add openrtm_common, rtmros_tutorials, rtmros_hironx, rtmros_gazebo, openrtm_apps, See Issue 137
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda, Satoshi Iwaishi, Shunichi Nozawa, furuta@jsk.imi.i.u-tokyo.ac.jp, garaemon@gmail.com, kei.okada, mmurooka, murooka@jsk.imi.i.u-tokyo.ac.jp, notheworld@gmail.com, s-noda@jsk.imi.i.u-tokyo.ac.jp, youhei@jsk.imi.i.u-tokyo.ac.jp
