(require :rtm-ros-robotics-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-model.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-applications.l")
(load "package://hrpsys_gazebo_atlas/euslisp/sandia-hand-interface.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-utils.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-walk.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-impedance-calib.l")

(ros::load-ros-manifest "hrpsys_gazebo_atlas")

;; load compiled messages
(setq ros::*compile-message* t)
(ros::roseus-add-msgs "visualization_msgs")
(ros::roseus-add-msgs "sensor_msgs")
(ros::load-ros-manifest "sandia_hand_msgs")
(ros::load-ros-manifest "atlas_msgs")
;;(ros::roseus-add-msgs "geometry_msgs")
(setq ros::*compile-message* nil)

;; mask vectors / reference atlas_msgs::AtlasState
(unless (constantp '*torso-mask-vector*)
  (defconstant *torso-mask-vector*
    (float-vector 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0)))
(unless (constantp '*head-mask-vector*)
  (defconstant *head-mask-vector*
    (float-vector 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0)))
(unless (constantp '*lleg-mask-vector*)
  (defconstant *lleg-mask-vector*
    (float-vector 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0)))
(unless (constantp '*rleg-mask-vector*)
  (defconstant *rleg-mask-vector*
    (float-vector 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0)))
(unless (constantp '*larm-mask-vector*)
  (defconstant *larm-mask-vector*
    (float-vector 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0)))
(unless (constantp '*rarm-mask-vector*)
  (defconstant *rarm-mask-vector*
    (float-vector 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1)))
(unless (constantp '*manip-mask-vector*)
  (defconstant *manip-mask-vector*
    (float-vector 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1)))
(unless (constantp '*stand-mask-vector*)
  (defconstant *stand-mask-vector*
    (float-vector 0 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0)))

(ros::roseus "atlas_interface")

(defclass atlas-interface
  :super rtm-ros-robot-interface
  :slots (l_hand_grasp_pub r_hand_grasp_pub bdi-action walk-seq vel-x-ratio vel-y-ratio vel-th-ratio))
(defmethod atlas-interface
  (:init (&optional (qualify nil))
   (prog1 (send-super :init :robot atlas-robot)
     ;; add controllers
     (dolist (l (list
                 (cons :larm-controller "larm_controller/follow_joint_trajectory_action")
                 (cons :rarm-controller "rarm_controller/follow_joint_trajectory_action")))
       (let ((type (car l))
             (name (cdr l))
             action)
         (setq action (find-if #'(lambda (ac) (string= name (send ac :name)))
                               controller-actions))
         (setf (gethash type controller-table) (list action))
         ))

     (setq l_hand_grasp_pub (ros::advertise "/sandia_hands/l_hand/simple_grasp"
                                            sandia_hand_msgs::SimpleGrasp 1)
           r_hand_grasp_pub (ros::advertise "/sandia_hands/r_hand/simple_grasp"
                                            sandia_hand_msgs::SimpleGrasp 1))
     (ros::advertise "atlas/control_mode" std_msgs::String 1)
     (ros::advertise "atlas/atlas_sim_interface_command"
                     atlas_msgs::AtlasSimInterfaceCommand 1)
     (ros::subscribe "atlas/atlas_sim_interface_state"
                     atlas_msgs::AtlasSimInterfaceState #'send self :sim-interface-state-cb)
     ;; (ros::subscribe "pointcloud_screenpoint_nodelet/output_point"
     ;;   geometry_msgs::PointStamped #'send self :point-stamped-cb)
     ;; do not use in Qualification
     (unless qualify
       (ros::advertise "atlas/mode" std_msgs::String 1)
       (ros::advertise "atlas/set_pose" geometry_msgs::Pose 1))
     ;; action lib / needs node of atlas_msgs/actionlib_server
     (setq bdi-action
           (instance ros::simple-action-client :init
                     "atlas/bdi_control" atlas_msgs::WalkDemoAction))
     (send bdi-action :wait-for-server 3)
     (setf (get robot :footstep-parameter)
           '(:default-half-offset #f(0.0 100.0 0.0) :forward-offset-length 180.0 :outside-offset-length 85.0 :rotate-rad 40.0))
     (setq walk-seq 0)
     (send self :set-recommended-walk-params)
     (setq vel-x-ratio 1.0 vel-y-ratio 1.0 vel-th-ratio 15.0)

     ;; additional filtered force sensor
     (ros::subscribe "rhand_filtered_wrench_handframe"
                     geometry_msgs::WrenchStamped
                     #'send self :rtmros-force-sensor-callback
                     :filtered-rhsensor :groupname groupname)
     (ros::subscribe "lhand_filtered_wrench_handframe"
                     geometry_msgs::WrenchStamped
                     #'send self :rtmros-force-sensor-callback
                     :filtered-lhsensor :groupname groupname)
     ))
  (:filtered-force-vector
   (&optional (limb))
   (send self :tmp-force-moment-vector :force limb "filtered"))
  (:filtered-moment-vector
   (&optional (limb))
   (send self :tmp-force-moment-vector :moment limb "filtered"))
  (:filtered-absolute-force-vector
   (&optional (limb))
   (if limb
       (send (send robot :force-sensor limb) :rotate-vector (send self :filtered-force-vector limb))
     (mapcar #'(lambda (fs force)
                 (if force (send fs :rotate-vector force)))
             (send robot :force-sensors) (send self :filtered-force-vector))))
  (:filtered-absolute-moment-vector
   (&optional (limb))
   (if limb
       (send (send robot :force-sensor limb) :rotate-vector (send self :filtered-moment-vector limb))
     (mapcar #'(lambda (fs moment)
                 (if moment (send fs :rotate-vector moment)))
             (send robot :force-sensors) (send self :filtered-moment-vector))))
  ;;
  ;; controller setting
  (:larm-controller ()
   (list
    (list
     (cons :controller-action "larm_controller/follow_joint_trajectory_action")
     (cons :controller-state "larm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "l_arm_shy" "l_arm_shx" "l_arm_ely" "l_arm_elx" "l_arm_wry" "l_arm_wrx")))))
  (:rarm-controller ()
   (list
    (list
     (cons :controller-action "rarm_controller/follow_joint_trajectory_action")
     (cons :controller-state "rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "r_arm_shy" "r_arm_shx" "r_arm_ely" "r_arm_elx" "r_arm_wry" "r_arm_wrx")))))
  (:lleg-controller ()
   (list
    (list
     (cons :controller-action "lleg_controller/follow_joint_trajectory_action")
     (cons :controller-state "lleg_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "l_leg_hpz" "l_leg_hpx" "l_leg_hpy" "l_leg_kny" "l_leg_aky" "l_leg_akx")))))
  (:rleg-controller ()
   (list
    (list
     (cons :controller-action "rleg_controller/follow_joint_trajectory_action")
     (cons :controller-state "rleg_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "r_leg_hpz" "r_leg_hpx" "r_leg_hpy" "r_leg_kny" "r_leg_aky" "r_leg_akx")))))
  (:head-controller ()
   (list
    (list
     (cons :controller-action "head_controller/follow_joint_trajectory_action")
     (cons :controller-state "head_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "neck_ry")))))
  (:torso-controller ()
   (list
    (list
     (cons :controller-action "torso_controller/follow_joint_trajectory_action")
     (cons :controller-state "torso_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "back_bkz" "back_bky" "back_bkx")))))
  (:default-controller ()
   (append
    (send self :larm-controller)
    (send self :rarm-controller)
    (send self :lleg-controller)
    (send self :rleg-controller)
    (send self :head-controller)
    (send self :torso-controller)
    ))
  ;;
  (:sim-interface-state-cb
   (msg)
   (let* ((pos-est (send msg :pos_est))
          (pos (send pos-est :position))
          (foot-pos-est1 (car (send msg :foot_pos_est)))
          (foot-pos-est2 (cadr (send msg :foot_pos_est)))
          (yaw (+ (acos (send (send foot-pos-est1 :orientation) :w))
                  (acos (send (send foot-pos-est2 :orientation) :w))))
          (cur-bhv (send msg :current_behavior)))
     (send self :put :pos-on-gazebo
           (scale 1000 (float-vector (send pos :x) (send pos :y) (send pos :z))))
     (send self :put :rotate-on-gazebo (rad2deg yaw))
     (send self :put :foot-estimation
           (list (ros::tf-pose->coords foot-pos-est1)
                 (ros::tf-pose->coords foot-pos-est2)))
     (send self :put :current-behavior cur-bhv)
     (send self :put :k-effort (send msg :k_effort))
     ))
  (:point-stamped-cb
   (msg)
   (print (string-left-trim "/" (send (send msg :header) :frame_id)))
   (if (member (string-left-trim "/" (send (send msg :header) :frame_id))
               (send-all (send *atlas* :links) :name)
               :test #'equal)
   (let* ((frame-id (send (send msg :header) :frame_id))
	  (point (send msg :point))
	  (frame (read-from-string (format nil ":~a" (string-left-trim "/" frame-id))))
	  (pos (scale 1000 (float-vector (send point :x) (send point :y) (send point :z)))))
     (print frame)
     (send *atlas* :angle-vector (send self :state :potentio-vector))
     (send self :put :point
	   (send (send *atlas* :worldcoords) :inverse-transform-vector (send (send (send *atlas* frame) :copy-worldcoords) :transform-vector pos)))
     )))
  (:wait-for-start-bdi-walk (&optional (timeout 5.0))
   (send self :wait-for-start-behavior 4 timeout))
  (:wait-for-bdi-walk (&optional (timeout 5.0))
   (send self :wait-for-finish-behavior 4 timeout))
  (:wait-for-start-behavior (&optional (behav 5) (timeout 5.0))
   (ros::spin-once)
   (while (and (not (equal (send self :get :current-behavior) behav))
               (> timeout 0.0))
     (ros::spin-once)
     (unix::usleep (* 100 1000))
     (decf timeout 0.1)
     ))
  (:wait-for-finish-behavior (&optional (behav 5))
   (ros::spin-once)
   (while (equal (send self :get :current-behavior) behav)
     (ros::spin-once)
     ))
  (:grasp
   (value &optional (arm :arms))
   (let ((msg (instance sandia_hand_msgs::SimpleGrasp :init :name "cylindrical" :closed_amount value)))
     (case arm
       (:arms
        (send self :grasp value :larm)
        (send self :grasp value :rarm))
       (:larm
        (ros::publish "/sandia_hands/l_hand/simple_grasp" msg))
       (:rarm
        (ros::publish "/sandia_hands/r_hand/simple_grasp" msg))
       )))
  (:start-grasp (&optional (arm :arms)) (send self :grasp 1 arm))
  (:stop-grasp  (&optional (arm :arms)) (send self :grasp 0 arm))
  (:control-mode (&optional (mode "Stand")) (send_atlas_control_mode mode))
  (:atlas-mode   (&optional (mode "nominal")) (send_atlas_mode mode))
  (:get-control-mode
   ()
   (ros::spin-once)
   (let* ((cb (send self :get :current-behavior))
	  ret)
     (cond
      ((equal cb atlas_msgs::AtlasSimInterfaceCommand::*STAND*) (setq ret :stand))
      ((equal cb atlas_msgs::AtlasSimInterfaceCommand::*FREEZE*) (setq ret :freeze))
      ((equal cb atlas_msgs::AtlasSimInterfaceCommand::*STAND_PREP*) (setq ret :stand-prep))
      ((equal cb atlas_msgs::AtlasSimInterfaceCommand::*WALK*) (setq ret :walk))
      ((equal cb atlas_msgs::AtlasSimInterfaceCommand::*STEP*) (setq ret :step))
      ((equal cb atlas_msgs::AtlasSimInterfaceCommand::*MANIPULATE*) (setq ret :manipulate))
      ((equal cb atlas_msgs::AtlasSimInterfaceCommand::*USER*) (setq ret :user))
      (t (setq ret :error)))
     ret))
  (:get-k-effort
   (&optional limb)
   (ros::spin-once)
   (if limb
       (apply #'subseq (concatenate cons (send self :get :k-effort))
	      (case limb (:torso '(0 3)) (:head '(3 4)) (:lleg '(4 10)) (:rleg '(10 16)) (:larm '(16 22)) (:rarm '(22 28))))
     (concatenate cons (send self :get :k-effort))))
  (:set-k-effort
   (&optional (e 0) (limb :all))
   (cond ((equal limb :all) (send self :set-simulation-mode :mode (send self :get-control-mode) :effort e))
	 ((equal limb :arms) (send self :set-k-effort e :larm) (send self :set-k-effort e :rarm))
	 ((equal limb :legs) (send self :set-k-effort e :lleg) (send self :set-k-effort e :rleg))
	 (t
	  (let* ((idx (case limb (:torso '(0 3)) (:head '(3 4)) (:lleg '(4 10)) (:rleg '(10 16)) (:larm '(16 22)) (:rarm '(22 28))))
		 (tmp-effort (send self :get-k-effort)))
	    (dotimes (i (- (elt idx 1) (elt idx 0)))
	      (setf (elt tmp-effort (+ (elt idx 0) i)) e))
	    (send self :set-simulation-mode :mode (send self :get-control-mode) :effort tmp-effort)))))
  (:set-simulation-mode
   (&rest args &key (effort 255) (sleep 0.5) (mode :user) (effort-mask)
          &allow-other-keys)
   (let ((msg (instance atlas_msgs::AtlasSimInterfaceCommand :init))
         (k-effort (instantiate string 28)))
     (case mode
       (:stand
        (setq mode atlas_msgs::AtlasSimInterfaceCommand::*STAND*))
       (:freeze
        (setq mode atlas_msgs::AtlasSimInterfaceCommand::*FREEZE*))
       (:stand-prep
        (setq mode atlas_msgs::AtlasSimInterfaceCommand::*STAND_PREP*))
       (:walk
        (setq mode atlas_msgs::AtlasSimInterfaceCommand::*WALK*))
       (:step
        (setq mode atlas_msgs::AtlasSimInterfaceCommand::*STEP*))
       (:manipulate
        (setq mode atlas_msgs::AtlasSimInterfaceCommand::*MANIPULATE*)
        (let ((p_des (position :use_desired args))
              (p_height (position :pelvis_height args))
              (p_yaw (position :pelvis_yaw args))
              (p_lat (position :pelvis_lat args)))
          (when p_des
            (send msg :manipulate_params :use_desired t))
          (when p_height
            (send msg :manipulate_params :desired :pelvis_height
                  (elt args (1+ p_height))))
          (when p_yaw
            (send msg :manipulate_params :desired :pelvis_yaw
                  (elt args (1+ p_yaw))))
          (when p_lat
            (send msg :manipulate_params :desired :pelvis_lat
                  (elt args (1+ p_lat)))))
        )
       ((:user t)
        (setq mode atlas_msgs::AtlasSimInterfaceCommand::*USER*)))
     (send msg :behavior mode)
     (cond ((not effort) ;; when effort is nil, not overwrite k-effort
            (ros::spin-once) (setq k-effort (send self :get :k-effort)))
           ((consp effort)
            (setq k-effort (concatenate string effort)))
           (t (fill k-effort effort)))
     (when effort-mask
       (setq k-effort
             (map string #'(lambda (val msk) (round (* val msk))) k-effort effort-mask)))
     (send msg :k_effort k-effort)
     (ros::publish "atlas/atlas_sim_interface_command" msg)
     (when sleep (unix::usleep (round (* sleep 1000 1000))))
     mode
     ))
  (:bdi_footstep
   (footstep-list &key (step-duration 0.63) (swing-height 0.3)
                  (foot-offset) &allow-other-keys)
   (let* ((fstep (car footstep-list)))
     (unless foot-offset
       (setq foot-offset (copy-object (send fstep :worldpos))))
     (setf (elt foot-offset 1) 0.0)
     (scale -1.0 foot-offset foot-offset)
     (let ((walk-goal (send bdi-action :make-goal-instance))
           (goal (instance atlas_msgs::WalkDemoGoal :init))
           ;;(step-params  (instance atlas_msgs::AtlasBehaviorStepParams :init))
           ;;(stand-params (instance atlas_msgs::AtlasBehaviorStandParams :init))
           ;;(manip-params (instance atlas_msgs::AtlasBehaviorManipulateParams :init))
           ;;(k-effort (instantiate integer-vector 28))
           (k-effort (instantiate string 28))
           steps (step-idx 0))
       ;; (fill k-effort 0)
       (ros::spin-once)
       (setq k-effort (send self :get :k-effort))
       ;; add first step ???
       (let* ((ffstep (copy-object (car footstep-list)))
              (step (instance atlas_msgs::AtlasBehaviorStepData
                              :init :step_index step-idx
                              :foot_index (if (eq (get ffstep :l/r) :rleg) 1 0) ;; :lleg -> 0 / :rleg -> 1
                              :duration 0
                              :swing_height 0)))
         (send ffstep :translate foot-offset :world)
         (send step :pose (ros::coords->tf-pose ffstep))
         (push step steps)
         (incf step-idx))
       ;;
       (dolist (fstep footstep-list)
         (let ((step (instance atlas_msgs::AtlasBehaviorStepData
                               :init :step_index step-idx
                               :foot_index (if (eq (get fstep :l/r) :rleg) 1 0) ;; :lleg -> 0 / :rleg -> 1
                               :duration step-duration
                               :swing_height swing-height)))
           (send fstep :translate foot-offset :world)
           (send step :pose (ros::coords->tf-pose fstep))
           (push step steps)
           (incf step-idx)
           ))
       ;;(send goal :header (instancd std_msgs::header :init))
       (send goal :behavior atlas_msgs::WalkDemoGoal::*WALK*)
       (send goal :k_effort k-effort)
       (send goal :steps (nreverse steps))
       (send walk-goal :header :seq walk-seq)
       (incf walk-seq)
       (send walk-goal :header :stamp (ros::time-now))
       (send goal :header :stamp (send walk-goal :header :stamp))
       (send walk-goal :goal_id :stamp (send walk-goal :header :stamp))
       (send walk-goal :goal goal)
       (send bdi-action :send-goal walk-goal)
       )))
  (:bdi_step_static (cds &key (l/r :lleg) ((:index idx) 0))
   (let ((walk-goal (send bdi-action :make-goal-instance))
         (goal (instance atlas_msgs::WalkDemoGoal :init))
         (k-effort (instantiate string 28)))
    (send goal :behavior atlas_msgs::WalkDemoGoal::*STEP*)
    ;;(send goal :step_params :use_demo_walk t)
    (send goal :step_params :desired_step :step_index idx)
    (send goal :step_params :desired_step :foot_index
          (case l/r (:lleg 0) (t 1)))  ;; :lleg -> 0 / :rleg -> 1
    (send goal :step_params :desired_step :duration 0.63) ;; not used??
    (send goal :step_params :desired_step :swing_height 0.3) ;; not used??
    (send goal :step_params :desired_step :pose (ros::coords->tf-pose cds))

    ;; (fill k-effort 0)
    (ros::spin-once)
    (setq k-effort (send self :get :k-effort))
    (send goal :k_effort k-effort)
    (send walk-goal :header :seq 1)
    (send walk-goal :header :stamp (ros::time-now))
    (send goal :header :stamp (send walk-goal :header :stamp))
    (send walk-goal :goal_id :stamp (send walk-goal :header :stamp))
    (send walk-goal :goal goal)
    (send bdi-action :send-goal walk-goal)
    ))
  (:bdi-go-pos (x y &optional (theta 0) &key (r robot) (step-duration 0.63) (swing-height 0.3))
               ;; x [m],  y[m], theta [deg]
   (send self :bdi_footstep
         (send robot :go-pos-params->footstep-list (* x 1000) (* y 1000) theta)
         :step-duration step-duration :swing-height swing-height))
  (:teleport-to (&optional coords pose keep-effort set-user-pinned)
   (unix::usleep (* 100 1000))
   (send self :atlas-mode "harnessed")
   (send self :set-simulation-mode :mode :freeze :effort (if keep-effort nil 0))
   (cond
    ((and pose (float-vector-p pose))
     (send robot :angle-vector pose)
     (send self :angle-vector (send robot :angle-vector) 2000)
     (send *ri* :set-simulation-mode :mode :stand-prep :effort (if keep-effort nil 0)))
    ((and pose (keywordp pose))
     (send robot pose)
     (send self :angle-vector (send robot :angle-vector) 2000)
     (send *ri* :set-simulation-mode :mode :stand-prep :effort (if keep-effort nil 0)))
    (t
     (send *ri* :set-simulation-mode :mode :stand-prep :effort (if keep-effort nil 0))
     ))
   (unix::usleep (* 3 1000 1000))
   (send self :atlas-mode "nominal")
   (unix::usleep (* 10 1000))
   (when coords
     (ros::publish "atlas/set_pose" (ros::coords->tf-pose coords)))
   (cond (set-user-pinned
	  (unix::usleep (* 200 1000))
	  (send self :atlas-mode "pinned")
	  (send self :set-simulation-mode :mode :user :effort (if keep-effort nil 255)))
	 (t
	  (unix::usleep (* 200 1000))
	  (send self :set-simulation-mode :mode :stand :effort (if keep-effort nil 0)))))
  ;;(:hand-move-joint
  ;;(arm av &optional (tm 1000))
  ;;)
  (:hand (arm &rest args)
   (let (hr ret)
     (case arm
       (:larm
        (setq hr (get self :lhand-controller))
        (setq ret (forward-message-to hr args)))
       (:rarm
        (setq hr (get self :rhand-controller))
        (setq ret (forward-message-to hr args)))
       (:both
        (setq hr (get self :lhand-controller))
        (push (forward-message-to hr args) ret)
        (setq hr (get self :rhand-controller))
        (push (forward-message-to hr args) ret))
       )
     ret))
  ;; hrpsys-base walk param
  (:set-recommended-walk-params
   ()
   (send self :set-auto-balancer-param
         :default-zmp-offsets (list (float-vector 40 0 0) (float-vector 40 0 0)))
   (send self :set-st-param
         :k-tpcc-p #f(2.5 2.5)
         :k-tpcc-x #f(5 5)
         ;;:k-brot-p #f(0.4 0.4)
	 :k-brot-p #f(5 5)
         :k-brot-tc #f(0.1 0.1))
   (send self :set-gait-generator-param
         :default-double-support-ratio 0.2
         :default-step-time 1.0
         :default-step-height 0.1
         ;;:stride-parameter #f(0.25 0.05 10)
	 :stride-parameter #f(0.25 0.05 30)
	 )
   )
  (:set-vel-param
   (xx yy th)
   (setq vel-x-ratio xx vel-y-ratio yy vel-th-ratio th))
  (:cmd-vel-cb
   (msg)
   (send-super :cmd-vel-cb
	       msg
	       :vel-x-ratio vel-x-ratio
	       :vel-y-ratio vel-y-ratio
	       :vel-th-ratio vel-th-ratio)
   )
  )
;;
(defmethod atlas-robot
  (:hand (arm &rest args)
   (let (hr ret)
     (case arm
       (:larm
        (setq hr (get self :lhand-model))
        (setq ret (forward-message-to hr args)))
       (:rarm
        (setq hr (get self :rhand-model))
        (setq ret (forward-message-to hr args)))
       (:both
        (setq hr (get self :lhand-model))
        (push (forward-message-to hr args) ret)
        (setq hr (get self :rhand-model))
        (push (forward-message-to hr args) ret))
       )
     ret))
  )

(defun send_atlas_control_mode
  (&optional (mode "Stand"))
  (let ((msg (instance std_msgs::String :init :data mode)))
    (ros::publish "atlas/control_mode" msg)
    (warn ";; send_atlas_control_mode is deprecated! use (send *ri* :set-simulation-mode :mode :mode_name) instead~%")))

;; do not use in Qualification
(defun send_atlas_mode
  (&optional (mode "nominal"))
  (let ((msg (instance std_msgs::String :init :data mode)))
    (ros::publish "atlas/mode" msg)))

(defun start-tilt-laser (&optional (speed 1.2))
  (one-shot-publish "/multisense_sl/set_spindle_speed" (instance std_msgs::Float64 :init :data speed)))

(defun get-tilt-laser (&key (remove-nan))
  (let ((msg (one-shot-subscribe "/full_cloud2" sensor_msgs::PointCloud2 :timeout 30000)))
    (when msg
      (let ((ret (make-eus-pointcloud-from-ros-msg msg :remove-nan remove-nan)))
        (setf (get ret :frame_id) (send msg :header :frame_id))
        (setf (get ret :stamp) (send msg :header :stamp))
        (setf (get ret :point_step) (send msg :point_step))
        ret
        ))))

(defun get-pos-estimation (&key (timeout 1000))
  (let ((msg (one-shot-subscribe "/atlas/atlas_sim_interface_state"
                                 atlas_msgs::AtlasSimInterfaceState :timeout timeout)))
    (list
     (list :pos
           (ros::tf-point->pos (send msg :pos_est :position)))
     (list :lfoot
           (ros::tf-pose->coords (car (send msg :foot_pos_est))))
     (list :rfoot
           (ros::tf-pose->coords (cadr (send msg :foot_pos_est))))
     )))

;;
;; footstep
;;
(defun generate-atlas-walk-pattern
  (&key (no-gen nil) (fname nil))
  (let (pattern-list
        (fname (if fname fname
		 (format nil "~A/atlas-walk-pattern"
			 (ros::resolve-ros-path "package://hrpsys_gazebo_atlas/build")))))
    (unless no-gen
      (format t ";; generate pattern~%")
      (setq pattern-list
	    (send *atlas* :calc-walk-pattern-from-footstep-list
		  (send *atlas* :go-pos-params->footstep-list
			500 150 45) ;; x[mm] y[mm] th[rad]
		  :debug-view :no-message :dt 0.01))
      (format t ";; dump pattern file~%")
      (send *ri* :dump-seq-pattern-file pattern-list fname))
    (format t ";; load pattern file~%")
    (send *ri* :loadPattern fname)
    ))

(defun atlas-walk-with-walk-pattern-gen ()
  (send_atlas_control_mode "Stand")
  (atlas-init :set-reset-pose nil)
  (objects (list *atlas*))
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
  (send_atlas_control_mode "User")
  (generate-atlas-walk-pattern)
  )

(defvar *footstep-planning-service* "/plan_footsteps")

(ros::load-ros-manifest "footstep_planner")

(defun make-ros-footstep (start goal)
  (let ((req (instance humanoid_nav_msgs::PlanFootstepsRequest :init)))
    ;;
    (let ((pos (send start :pos))
          (theta (caar (send start :rpy-angle))))
      (send req :start :x (* 1e-3 (elt pos 0)))
      (send req :start :y (* 1e-3 (elt pos 1)))
      (send req :start :theta theta))
    ;;
    (let ((pos (send goal :pos))
          (theta (caar (send goal :rpy-angle))))
      (send req :goal :x (* 1e-3 (elt pos 0)))
      (send req :goal :y (* 1e-3 (elt pos 1)))
      (send req :goal :theta theta))
    ;;
    (ros::service-call *footstep-planning-service* req)
    ))

(defun footstep-planning-armin
  (start goal)
  (let* ((res
          (make-ros-footstep start goal))
         (path
          (mapcar
           #'(lambda (msg)
	       (let (coords)
		 (setq coords (make-coords :pos (float-vector
						 (* 1000 (send msg :pose :x))
						 (* 1000 (send msg :pose :y)) 0)))
		 (send coords :rotate (send msg :pose :theta) :z)
		 (setq coords (send start :transformation coords))
		 (send coords :put :l/r
		       (cond
			((= (send msg :leg) humanoid_nav_msgs::StepTarget::*left*) :lleg)
			((= (send msg :leg) humanoid_nav_msgs::StepTarget::*right*) :rleg)))
		 coords
		 ))
           (send res :footsteps))))
    ;; append first 1 or 2 step(s)
    path))
;; (send *ri* :bdi_footstep (footstep-planning-armin (make-coords :pos (scale 1000 (float-vector 0.5 1.55 0)) :rpy (list 0 0 0)) (make-coords :pos (scale 1000 (float-vector 6.2 1.55 0)) :rpy (list 0 0 0))))
;; (send *ri* :bdi_footstep (footstep-planning-armin (make-coords :pos (scale 1000 (float-vector 1 1 0)) :rpy (list 0 0 0)) (make-coords :pos (scale 1000 (float-vector 17.3 9 0)) :rpy (list 0 0 0))))

(defun atlas-init
  (&key (set-reset-pose nil) (set-user-mode nil) (unset-limit t) (with-hand nil))
  (if (not (boundp '*ri*)) (setq *ri* (instance atlas-interface :init)))
  ;; (if (not (boundp '*ri*)) (setq *ri* (instance atlas-interface :init :use-tf2 t)))
  (if (not (boundp '*atlas*)) (setq *atlas* (atlas-with-hand)))
  ;; parameter for bdi_control
  (setf (get *atlas* :footstep-parameter)
       '(:default-half-offset #f(0.0 100.0 0.0) :forward-offset-length 180.0 :outside-offset-length 85.0 :rotate-rad 40.0))
  (when unset-limit
    (send *ri* :set-soft-error-limit "all" 100) ;; disable SoftErrorLimiter
    (send *ri* :set-servo-error-limit "all" 0) ;; disable RobotHardware error limit
    )
  (when set-user-mode
    (send *ri* :set-simulation-mode))
  (when set-reset-pose
    (cond
     ((keywordp set-reset-pose)
      (send *atlas* set-reset-pose))
     (t (send *atlas* :reset-pose)
        (send *atlas* :move-centroid-on-foot :both '(:lleg :rleg))))
    ;; send reset pose to real robot
    (send *ri* :angle-vector (send *atlas* :angle-vector) 3000)
    (send *ri* :wait-interpolation))
  (imu-init)                  ;;subscribe /atlus/imu
  (forcetorque-init)          ;;subscribe /atlus_msg/ForceTorqueSensor
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (when with-hand
    (setf (get *ri* :lhand-controller)
          (instance sandia-hand-interface :init :larm))
    (setf (get *ri* :rhand-controller)
          (instance sandia-hand-interface :init :rarm)))
  *ri*
  )
#|
  for utility
|#
(defun send-pose (&optional (tm 3000))
  (send *ri* :angle-vector (send *atlas* :angle-vector) tm))
(defun adapt-pose ()
  (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
  (send *irtviewer* :draw-objects))
;; (defun reset-pose ()
;;   (send *atlas* :reset-pose))

#|
  ROS-API経由でatlasのセンサ情報を取得
  /atlas/imu/orientation x,y,z,wで読める
|#
(defun imu-init ()
  (ros::subscribe "/atlas/imu" sensor_msgs::Imu #'get_imu_quaternion 1))

(defun get-imu-data (&optional (timeout 1000))
  (setq *atlas_rpy* nil)
  (ros::spin-once)
  (unless *atlas_rpy*
    (let ((endtm (ros::time+ (ros::time-now) (ros::time (/ timeout 1000.0)))))
      (while (null *atlas_rpy*)
        (ros::spin-once)
        (if (< (send (ros::time- endtm (ros::time-now)) :to-sec) 0.0)
            (return))
        )))
  *atlas_rpy*
  )

(defvar *atlas_rpy* nil)
;;ros:spin-onceすれば値が更新される
(defun get_imu_quaternion (msg)
  (let* ((q (send msg :orientation)))
    ;;(print imu-orientation)
    (setq *atlas_rpy*
          (mapcar
           #'rad2deg
           (car (rpy-angle (quaternion2matrix
                            (float-vector (send q :w)
                                          (send q :x)
                                          (send q :y)
                                          (send q :z)))))))
    ))

(defun forcetorque-init ()
  (ros::subscribe "/atlas/force_torque_sensors"
                  atlas_msgs::ForceTorqueSensors #'get_forcetorque_sensors 1))

(defun get-forcetorque-data (&optional (timeout 1000))
  (setq *atlas_forcetorque* nil)
  (ros::spin-once)
  (unless *atlas_forcetorque*
    (let ((endtm (ros::time+ (ros::time-now) (ros::time (/ timeout 1000.0)))))
      (while (null *atlas_forcetorque*)
        (ros::spin-once)
        (if (< (send (ros::time- endtm (ros::time-now)) :to-sec) 0.0)
            (return))
        )))
  *atlas_forcetorque*
  )

(defvar *atlas_forcetorque* nil)
(defun get_forcetorque_sensors (msg)
  (let* ((lf (send msg :l_foot))
     (rf (send msg :r_foot))
     (la (send msg :l_hand))
     (ra (send msg :r_hand))
     (ret))

    (setq *atlas_forcetorque*
      (list
       (cons :l_foot
         (float-vector (send lf :force :x)
                   (send lf :force :y)
                   (send lf :force :z)
                   (send lf :torque :x)
                   (send lf :torque :y)
                   (send lf :torque :z)))
       (cons :r_foot
         (float-vector (send rf :force :x)
                   (send rf :force :y)
                   (send rf :force :z)
                   (send rf :torque :x)
                   (send rf :torque :y)
                   (send rf :torque :z)))
       (cons :l_hand
         (float-vector (send la :force :x)
                   (send la :force :y)
                   (send la :force :z)
                   (send la :torque :x)
                   (send la :torque :y)
                   (send la :torque :z)))
       (cons :r_hand
         (float-vector (send ra :force :x)
                   (send ra :force :y)
                   (send ra :force :z)
                   (send ra :torque :x)
                   (send ra :torque :y)
                   (send ra :torque :z)))
       ))))

;; degで指定した角度でリミットトルクが出るようにゲインを設定
#|
  /opt/ros/groovy/share/drcsim_gazebo/config/whole_body_trajectory_controller_v3.yaml
  gains:
    l_leg_hpz: {p:     5.0, d:  0.01, i:   0.0, i_clamp:    0.0}
    l_leg_hpx: {p:   100.0, d:   1.0, i:   0.0, i_clamp:    0.0}
    l_leg_hpy: {p:  2000.0, d:  10.0, i:   0.0, i_clamp:    0.0}
    l_leg_kny: {p:  1000.0, d:  10.0, i:   0.0, i_clamp:    0.0}
    l_leg_aky: {p:   900.0, d:   2.0, i:   0.0, i_clamp:    0.0}
    l_leg_akx: {p:   300.0, d:   1.0, i:   0.0, i_clamp:    0.0}
    r_leg_hpz: {p:     5.0, d:  0.01, i:   0.0, i_clamp:    0.0}
    r_leg_hpx: {p:   100.0, d:   1.0, i:   0.0, i_clamp:    0.0}
    r_leg_hpy: {p:  2000.0, d:  10.0, i:   0.0, i_clamp:    0.0}
    r_leg_kny: {p:  1000.0, d:  10.0, i:   0.0, i_clamp:    0.0}
    r_leg_aky: {p:   900.0, d:   2.0, i:   0.0, i_clamp:    0.0}
    r_leg_akx: {p:   300.0, d:   1.0, i:   0.0, i_clamp:    0.0}
    l_arm_shy: {p:  2000.0, d:   3.0, i:   0.0, i_clamp:    0.0}
    l_arm_shx: {p:  1000.0, d:  20.0, i:   0.0, i_clamp:    0.0}
    l_arm_ely: {p:   200.0, d:   3.0, i:   0.0, i_clamp:    0.0}
    l_arm_elx: {p:   200.0, d:   3.0, i:   0.0, i_clamp:    0.0}
    l_arm_wry: {p:    50.0, d:   0.1, i:   0.0, i_clamp:    0.0}
    l_arm_wrx: {p:   100.0, d:   0.2, i:   0.0, i_clamp:    0.0}
    r_arm_shy: {p:  2000.0, d:   3.0, i:   0.0, i_clamp:    0.0}
    r_arm_shx: {p:  1000.0, d:  20.0, i:   0.0, i_clamp:    0.0}
    r_arm_ely: {p:   200.0, d:   3.0, i:   0.0, i_clamp:    0.0}
    r_arm_elx: {p:   200.0, d:   3.0, i:   0.0, i_clamp:    0.0}
    r_arm_wry: {p:    50.0, d:   0.1, i:   0.0, i_clamp:    0.0}
    r_arm_wrx: {p:   100.0, d:   0.2, i:   0.0, i_clamp:    0.0}
    neck_ry:   {p:    20.0, d:   1.0, i:   0.0, i_clamp:    0.0}
    back_bkz:  {p:    20.0, d:   0.1, i:   0.0, i_clamp:    0.0}
    back_bky:  {p:  4000.0, d:  02.0, i:   0.0, i_clamp:    0.0}
    back_bkx:  {p:  2000.0, d:  01.0, i:   0.0, i_clamp:    0.0}

    joint name="back_bkx"   effort="200"
    joint name="back_bky"   effort="206.84"
    joint name="back_bkz"   effort="124.016"
    joint name="l_arm_elx"  effort="114"
    joint name="l_arm_ely"  effort="114"
    joint name="l_arm_shx"  effort="170"
    joint name="l_arm_shy"  effort="212"
    joint name="l_arm_wrx"  effort="60"
    joint name="l_arm_wry"  effort="114"
    joint name="l_leg_akx"  effort="90"
    joint name="l_leg_aky"  effort="700"
    joint name="l_leg_hpx"  effort="180"
    joint name="l_leg_hpy"  effort="260"
    joint name="l_leg_hpz"  effort="110"
    joint name="l_leg_kny"  effort="220"
    joint name="neck_ry"    effort="5"
    joint name="r_arm_elx"  effort="114"
    joint name="r_arm_ely"  effort="114"
    joint name="r_arm_shx"  effort="170"
    joint name="r_arm_shy"  effort="212"
    joint name="r_arm_wrx"  effort="60"
    joint name="r_arm_wry"  effort="114"
    joint name="r_leg_akx"  effort="90" -> (rad2deg (* 100 (/ 90.0[Nm] 300[Nm/rad]))) -> 1718.87
    joint name="r_leg_aky"  effort="700"
    joint name="r_leg_hpx"  effort="180"
    joint name="r_leg_hpy"  effort="260"
    joint name="r_leg_hpz"  effort="110"
    joint name="r_leg_kny"  effort="220"

   '(((:name 'l_leg_hpz) (:p     5.0) (:effort    110))
     ((:name 'l_leg_hpx) (:p   100.0) (:effort    180))
     ((:name 'l_leg_hpy) (:p  2000.0) (:effort    260))
     ((:name 'l_leg_kny) (:p  1000.0) (:effort    220))
     ((:name 'l_leg_aky) (:p   900.0) (:effort    700))
     ((:name 'l_leg_akx) (:p   300.0) (:effort     90))
     ((:name 'r_leg_hpz) (:p     5.0) (:effort    110))
     ((:name 'r_leg_hpx) (:p   100.0) (:effort    180))
     ((:name 'r_leg_hpy) (:p  2000.0) (:effort    260))
     ((:name 'r_leg_kny) (:p  1000.0) (:effort    220))
     ((:name 'r_leg_aky) (:p   900.0) (:effort    700))
     ((:name 'r_leg_akx) (:p   300.0) (:effort     90))
     ((:name 'l_arm_shy) (:p  2000.0) (:effort    212))
     ((:name 'l_arm_shx) (:p  1000.0) (:effort    170))
     ((:name 'l_arm_ely) (:p   200.0) (:effort    114))
     ((:name 'l_arm_elx) (:p   200.0) (:effort    114))
     ((:name 'l_arm_wry) (:p    50.0) (:effort    114))
     ((:name 'l_arm_wrx) (:p   100.0) (:effort     60))
     ((:name 'r_arm_shy) (:p  2000.0) (:effort    212))
     ((:name 'r_arm_shx) (:p  1000.0) (:effort    170))
     ((:name 'r_arm_ely) (:p   200.0) (:effort    114))
     ((:name 'r_arm_elx) (:p   200.0) (:effort    114))
     ((:name 'r_arm_wry) (:p    50.0) (:effort    114))
     ((:name 'r_arm_wrx) (:p   100.0) (:effort     60))
     ((:name 'neck_ry)   (:p    20.0) (:effort      5))
     ((:name 'back_bkz)  (:p    20.0) (:effort    124.016))
     ((:name 'back_bky)  (:p  4000.0) (:effort    206.84))
     ((:name 'back_bkx)  (:p  2000.0) (:effort    200)))

    (mapcar #'(lambda (param) (let ((name (cadr (assoc :name param)))(gain (cadr (assoc :p param))) (limit (cadr (assoc :effort param))) x) (setq x (rad2deg (* 100 (/ limit gain)))) (print (list name limit gain x)) x)) daa)
|#

(defun atlas-set-servo-gain-by-torque-limit
  (&optional (deg 1) (limb :all))
  (if (not (memq limb '(:all :torso :head :lleg :rleg :larm :rarm :legs :arms)))
      (ros::ros-error "atlas-set-servo-gain-by-torque-limit failed, unknown limb ~A" limb))
  (when (or (equal limb :all) (equal limb :torso))
    (send *ri* :set-servo-gain-percentage "back_bkz"  (/ 35528  deg))
    (send *ri* :set-servo-gain-percentage "back_bky"  (/ 296    deg))
    (send *ri* :set-servo-gain-percentage "back_bkx"  (/ 572    deg)))
  (when (or (equal limb :all) (equal limb :head))
    (send *ri* :set-servo-gain-percentage "neck_ry"   (/ 1432   deg)))
  (when (or (equal limb :all) (equal limb :lleg) (equal limb :legs))
    (send *ri* :set-servo-gain-percentage "l_leg_hpz" (/ 126051 deg))
    (send *ri* :set-servo-gain-percentage "l_leg_hpx" (/ 10313  deg))
    (send *ri* :set-servo-gain-percentage "l_leg_hpy" (/ 745    deg))
    (send *ri* :set-servo-gain-percentage "l_leg_kny" (/ 1261   deg))
    (send *ri* :set-servo-gain-percentage "l_leg_aky" (/ 4456   deg))
    (send *ri* :set-servo-gain-percentage "l_leg_akx" (/ 1719   deg)))
  (when (or (equal limb :all) (equal limb :rleg) (equal limb :legs))
    (send *ri* :set-servo-gain-percentage "r_leg_hpz" (/ 126051 deg))
    (send *ri* :set-servo-gain-percentage "r_leg_hpx" (/ 10313  deg))
    (send *ri* :set-servo-gain-percentage "r_leg_hpy" (/ 745    deg))
    (send *ri* :set-servo-gain-percentage "r_leg_kny" (/ 1261   deg))
    (send *ri* :set-servo-gain-percentage "r_leg_aky" (/ 4456   deg))
    (send *ri* :set-servo-gain-percentage "r_leg_akx" (/ 1719   deg)))
  (when (or (equal limb :all) (equal limb :larm) (equal limb :arms))
    (send *ri* :set-servo-gain-percentage "l_arm_shy" (/ 607    deg))
    (send *ri* :set-servo-gain-percentage "l_arm_shx" (/ 974    deg))
    (send *ri* :set-servo-gain-percentage "l_arm_ely" (/ 3266   deg))
    (send *ri* :set-servo-gain-percentage "l_arm_elx" (/ 3266   deg))
    (send *ri* :set-servo-gain-percentage "l_arm_wry" (/ 13063  deg))
    (send *ri* :set-servo-gain-percentage "l_arm_wrx" (/ 3438   deg)))
  (when (or (equal limb :all) (equal limb :rarm) (equal limb :arms))
    (send *ri* :set-servo-gain-percentage "r_arm_shy" (/ 607    deg))
    (send *ri* :set-servo-gain-percentage "r_arm_shx" (/ 974    deg))
    (send *ri* :set-servo-gain-percentage "r_arm_ely" (/ 3266   deg))
    (send *ri* :set-servo-gain-percentage "r_arm_elx" (/ 3266   deg))
    (send *ri* :set-servo-gain-percentage "r_arm_wry" (/ 13063  deg))
    (send *ri* :set-servo-gain-percentage "r_arm_wrx" (/ 3438   deg)))
  )

;; usage ;; (fix-leg-strigde (float-vector 0 20 0))
(defun modify-leg-stride (offset/2)
  (send *atlas* :gen-footstep-parameter)
  (let* ((midp (apply #'midpoint 0.5 (send *atlas* :legs :end-coords :worldpos)))
         (diff (cadr (memq :default-half-offset (send *atlas* :get :footstep-parameter)))) ;; reset-pose stride
         (diff2 (v+ diff offset/2))
         (fs (list (make-coords :pos (v+ midp diff2))
                   (make-coords :pos (v- midp diff2))
                   (make-coords :pos (v+ midp diff2)))))
    (send (elt fs 0) :put :l/r :lleg)
    (send (elt fs 1) :put :l/r :rleg)
    (send (elt fs 2) :put :l/r :lleg)
    (send *ri* :start-auto-balancer)
    (send *ri* :set-foot-steps-no-wait fs)
    (send *ri* :wait-foot-steps)
    (send *atlas* :angle-vector (send *ri* :state :potentio-vector))
    (send *atlas* :fix-leg-to-coords (make-coords) '(:rleg :lleg))
    (send *ri* :angle-vector (send *atlas* :angle-vector) 1000)
    (send *ri* :wait-interpolation)
    (send *ri* :stop-auto-balancer)))

(defun reset-bdi-controller ()
  (let ((req (instance atlas_msgs::ResetControlsRequest :init)))
    (send req :reset_bdi_controller t)
    (send req :reload_pid_from_ros t)
    (ros::service-call "/atlas/reset_controls" req)
    ))

(defun static-step (&key (l/r :rleg) (step 300)) ;; should be stand-pose
  (reset-bdi-controller)
  (unix::usleep (* 500 1000))
  (send *ri* :set-simulation-mode :mode :stand :effort 0)
  (unix::usleep (* 1000 1000))
  (case l/r
   (:rleg
    (send *ri* :bdi_step_static (make-coords :pos (float-vector step -140 0)) :l/r :rleg :index 0))
   (:lleg
    (send *ri* :bdi_step_static (make-coords :pos (float-vector step 140 0)) :l/r :lleg :index 0)))
  (send *ri* :wait-for-start-behavior 5)
  (send *ri* :wait-for-finish-behavior 5)
  (case l/r
   (:rleg
    (send *ri* :bdi_step_static (make-coords :pos (float-vector (/ step 2) 140 0)) :l/r :lleg :index 1))
   (:lleg
    (send *ri* :bdi_step_static (make-coords :pos (float-vector (/ step 2) -140 0)) :l/r :rleg :index 1)))
  )

(defmethod atlas-robot
  (:pre-manip-pose ()
   (send self :angle-vector
         (float-vector 0.0 3.0 0.0 110.0 120.0 60.0 -70.0 45.0 0.0 -110.0 120.0 -60.0 70.0 45.0 0.0 28.0 -1.7 -12.5 1.7 -15.5 0.0 28.0 1.7 -12.5 -1.7 -15.5 0.0 0.0)))
  )
(in-package "ROS")
(defmethod ros::simple-action-client
  (:wait-for-server
   (&optional (timeout nil))
   (let ((goal-topic (format nil "~A/goal" name-space))
         (cancel-topic (format nil "~A/cancel" name-space))
         (feedback-topic (format nil "~A/feedback" name-space))
         (result-topic (format nil "~A/result" name-space))
         (count 0))
     (ros::ros-debug "[~A] wait-for-server" name-space)
     (ros::rate 10)
     (while (or (= (ros::get-num-subscribers goal-topic) 0)
                (= (ros::get-num-subscribers cancel-topic) 0)
                ;;(= (ros::get-num-publishers feedback-topic) 0) ;; by pragmatic reason
                (= (ros::get-num-publishers result-topic) 0))
       (ros::ros-debug "[~A] num subscribers ~A ~d, ~A ~d" name-space
                       goal-topic (ros::get-num-subscribers goal-topic)
                       cancel-topic (ros::get-num-subscribers cancel-topic))
       (send self :spin-once)
       (ros::sleep)
       (when (and timeout (>= count (* timeout 10)))
         (ros::ros-warn "[~A] action server is not found" name-space)
         (ros::ros-warn "     goal=~d, cancel=~d, feedback=~d, result=~d"
                        (ros::get-num-subscribers goal-topic)
                        (ros::get-num-subscribers cancel-topic)
                        (ros::get-num-publishers feedback-topic)
                        (ros::get-num-publishers result-topic))
         (return-from :wait-for-server nil))
       (incf count))
     t))
  )
(in-package "USER")
