#!/usr/bin/env roseus

(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "sandia_hand_msgs")
(ros::load-ros-manifest "humanoid_nav_msgs")
(ros::load-ros-manifest "atlas_msgs")
(ros::load-ros-manifest "arm_navigation_msgs")
(ros::load-ros-manifest "moveit_msgs")
(require :rtm-ros-robotics-interface "package://hrpsys_ros_bridge/euslisp/rtm-ros-robot-interface.l")

(load "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(load "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")


(ros::roseus "atlas_eus_ik")


(defun setup-end-coords ()
  ;; right
  (let ((end-coords (make-coords)) 
        (arm :rarm)
        ;;(palm-link :right_palm_lk)
        (palm-link :right_palm_left_camera_optical_frame_lk)
        (axis (float-vector 0 0 1))
        (rad 2.23))
        ;;(rad 1.57))
    ;;(rad 0))
    (send end-coords :rotate rad axis)
    (send end-coords :translate (float-vector 30 -250 45) :world)
    (let ((new-end-coords
           (send (send *atlas* :r_hand_lk) :copy-worldcoords)))
      ;;(send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
      (send new-end-coords :transform end-coords :local)
      (let ((trans (send (send *atlas* arm :end-coords :copy-worldcoords)
                         :transformation new-end-coords)))
        (send (send *atlas* arm :end-coords) :transform trans))
      ))
  ;; left
  (let ((end-coords (make-coords)) 
        (arm :larm)
        ;;(palm-link :right_palm_lk)
        (palm-link :left_palm_left_camera_optical_frame_lk)
        (axis (float-vector 0 0 1))
        (rad 0.91))
    (send end-coords :rotate rad axis)
    (send end-coords :translate (float-vector 30 250 45) :world)
    (let ((new-end-coords
           (send (send *atlas* :l_hand_lk) :copy-worldcoords)))
           ;;(send (send (send *atlas* :hand arm) palm-link) :copy-worldcoords)))
      (send new-end-coords :transform end-coords :local)
      (let ((trans (send (send *atlas* arm :end-coords) :transformation
                   new-end-coords)))
        (send (send *atlas* arm :end-coords) :transform trans))
      ))
  )

;(defun setup-end-coords nil nil)

(defun fullbody-ik (arm goal-in-world)
  (send *atlas* :fullbody-inverse-kinematics
        (list goal-in-world
              (copy-object (send *atlas* :rleg :end-coords :worldcoords))
              (copy-object (send *atlas* :lleg :end-coords :worldcoords)))
        :move-target (mapcar
                      #'(lambda (k) (send *atlas* k :end-coords))
                      (list arm :rleg :lleg))
        :root-link-virtual-joint-weight
        (float-vector 0.01 0.01 1 0.01 0.01 0.01)
        :min (float-vector -1000 -1000 -1000 -200 -200 -200)
        :max (float-vector 1000 1000 1000 200 200 200)
        :link-list (mapcar
                    #'(lambda (k)
                        (send *atlas* :link-list
                              (send (send *atlas* k :end-coords) :parent)))
                    (list arm :rleg :lleg))
        :debug-view :no-message
        :warnp t
        :centroid-thre 100
        :thre '(1 1 1)
        :rotation-axis '(t t t)))

(defun fullbody-ik-with-collision (arm goal-in-world)
  (fullbody-ik-main
   :target-limb (list arm :lleg :rleg)
   :target-coords (list goal-in-world nil nil)
   :collision? t
   :centroid-thre 100
   :thre '(1 1 1)
   :rotation-axis '(t t t)))

;; (defun torso-ik (arm goal-in-world)
;;   (let ((link-list (send *atlas* :link-list (send (send *atlas* arm :end-coords) :parent))))
;;     (send *atlas* arm
;;            :inverse-kinematics goapppl-in-world
;;            :stop 200
;;            :debug-view :no-message
;;            :warnp nil
;;            :thre 1
;;            :rthre (deg2rad 1)
;;            :additional-weight-list
;;            (mapcar
;;             #'(lambda (k gain)
;;                 (list (send *atlas* :torso k :child-link)
;;                       gain))
;;             (list :waist-r :waist-p :waist-y)
;;                                         ;(list 1e-3 1e-3 1e-3))
;;             (list 1e-10 1e-6 1e-5))
;;            :link-list link-list
;;            :rotation-axis t))) ;default it uses torso

(defun torso-ik (arm goal-in-world)
  (send *atlas* :fullbody-inverse-kinematics
        (list goal-in-world
              (copy-object (send *atlas* :rleg :end-coords :worldcoords))
              (copy-object (send *atlas* :lleg :end-coords :worldcoords)))
        :move-target (mapcar
                      #'(lambda (k) (send *atlas* k :end-coords))
                      (list arm :rleg :lleg))
        :additional-weight-list
           (mapcar
            #'(lambda (k gain)
                (list (send *atlas* :torso k :child-link)
                      gain))
            (list :waist-r :waist-p :waist-y)
                                        ;(list 1e-3 1e-3 1e-3))
	    (list 0.1 0.1 0.1))
            ;;(list 1e-3 1e-3 1e-3))
        :root-link-virtual-joint-weight
        (float-vector 0 0 0 0 0 0)
        :min (float-vector 0 0 0 0 0 0)
        :max (float-vector 0 0 0 0 0 0)
        :link-list (mapcar
                    #'(lambda (k)
                        (send *atlas* :link-list
                              (send (send *atlas* k :end-coords) :parent)))
                    (list arm :rleg :lleg))
        :debug-view :no-message
        :warnp t
        :centroid-thre 100
        :thre '(5 5 1)
        :rotation-axis '(t t t)))

(defun torso-ik-with-collision (arm goal-in-world)
  (fullbody-ik-main
   :target-limb (list arm :lleg :rleg)
   :target-coords (list goal-in-world nil nil)
   :additional-weight-list
   (mapcar
    #'(lambda (k gain)
	(list (send *atlas* :torso k :child-link)
	      gain))
    (list :waist-r :waist-p :waist-y)
    (list 0.1 0.1 0.1))
   ;;(list 1e-3 1e-3 1e-3))
   :root-link-virtual-joint-weight
   (float-vector 0 0 0 0 0 0)
   :min-vec (float-vector 0 0 0 0 0 0)
   :max-vec (float-vector 0 0 0 0 0 0)
   :collision? t
   :centroid-thre 200
   :thre '(5 5 1)
   :rotation-axis '(t t t)))


(defun normal-ik (arm goal-in-world)
  (send *atlas* arm :inverse-kinematics goal-in-world :stop 200
        :thre 1
        :rthre (deg2rad 1)
        :debug-view :no-message :warnp nil))




(defun ik-service-cb (req)
  (let ((res (send req :response))
        (arm (if (or (string= (subseq (send req :ik_request :group_name) 0 4) "larm")
                     (string= (subseq (send req :ik_request :group_name) 0 4) "lhand"))
                 :larm
               :rarm))
        (torsop (or (string= (send req :ik_request :group_name) "larm_torso")
                    (string= (send req :ik_request :group_name) "rarm_torso")))
        (fullbodyp (or (string= (send req :ik_request :group_name) "larm_fullbody")
                       (string= (send req :ik_request :group_name) "rarm_fullbody"))))
    ;; taking joint state into account
    (if (and (send req :ik_request :robot_state)
               (send req :ik_request :robot_state :joint_state)
               (send req :ik_request :robot_state :joint_state :name)
               (send req :ik_request :robot_state :joint_state :position)
               (= (length (send req :ik_request :robot_state :joint_state :name))
                  (length (send req :ik_request :robot_state :joint_state :position))))
        (progn
          (dotimes (i (length (send req :ik_request :robot_state :joint_state :name)))
            (let ((joint-name (elt (send req :ik_request :robot_state :joint_state :name) i)))
              (let ((j (find joint-name (send *atlas* :joint-list) :test #'string= :key #'(lambda (x) (send x :name)))))
                (if j
                    (send j :joint-angle (rad2deg (elt (send req :ik_request :robot_state :joint_state :position) i)))
                  (warn "we cannot find ~A~%" joint-name)
                  )))))
      (send *atlas* :reset-manip-pose)
      )
    (send *viewer* :draw-objects)
    (send *atlas* :newcoords (make-coords)) ;force to move pelvis origin
    (let ((goal (send req :ik_request :pose_stamped)))
      ;; check that goal is malformed or not
      (let ((q (float-vector (send req :ik_request :pose_stamped :pose :orientation :w)
                             (send req :ik_request :pose_stamped :pose :orientation :x)
                             (send req :ik_request :pose_stamped :pose :orientation :y)
                             (send req :ik_request :pose_stamped :pose :orientation :z))))
        (if (not (eps= (norm q) 1.0))
            (return nil)))
      (let ((goal-in-world (send *tf* :transform-pose "/pelvis" goal)))
        (let ((ik-ret (cond
                       (fullbodyp
                        ;; (fullbody-ik arm goal-in-world)
                        (fullbody-ik-with-collision arm goal-in-world)
			)
                       (torsop
                        ;;(torso-ik arm goal-in-world))
                        (torso-ik-with-collision arm goal-in-world))
                       (t
                        (normal-ik arm goal-in-world)))))
          (if ik-ret
              (progn
                (send res :error_code :val moveit_msgs::MoveItErrorCodes::*SUCCESS*)
                (send res :solution :joint_state :name (send-all (send *atlas* :joint-list) :name))
                (send res :solution :joint_state :position
                      (map float-vector #'deg2rad (send *atlas* :angle-vector)))
                )
            (progn
              (send res :error_code :val moveit_msgs::MoveItErrorCodes::*FAILURE*)))
          res)))))

;; use transpose
;; (defmethod cascaded-link
;;   (:calc-inverse-jacobian
;;     (jacobi &rest args
;;             &key
;;             ((:manipulability-limit ml) 0.1)
;;             ((:manipulability-gain mg) 0.001)
;;             weight debug-view
;;             ret wmat tmat umat umat2 mat-tmp
;;             mat-tmp-rc tmp-mrr tmp-mrr2
;;             &allow-other-keys)
;;     (let (jacobi# m m2 (k 0))
;;       ;; m : manipulability
;;       (setq m (manipulability jacobi tmp-mrr tmat))
;;       (if (< m ml) (setq k (* mg (expt (- 1.0 (/ m  ml)) 2))))
;;       (when (and debug-view (not (memq :no-message debug-view)))
;;         (warn "k     :~7,3f (manipulability:~7,3f, gain:~7,3f, limit:~7,3f, len:~d)
;; ~%" k m mg ml (cadr (array-dimensions jacobi))))
      
;;       ;; calc weighted SR-inverse
;;       ;; (setq jacobi# (sr-inverse jacobi k weight
;;       ;;                           ret wmat tmat umat umat2 mat-tmp
;;       ;;                           mat-tmp-rc tmp-mrr tmp-mrr2
;;       ;;                           ))
;;       ;; jacobi#
;;       ;;(dotimes (i (array-dimension jacobi 1))
;;       ;;(setf (aref wmat i i) (aref weight-vector i)))
;;       ;;(transpose (m* wmat jacobi))
;;       (transpose jacobi))))

;; initialization
(setq *atlas* (atlas-with-hand))
(send-all (send *atlas* :joint-list) :max-joint-velocity 0.3)
(setup-end-coords)
(objects (list *atlas*))
(send *irtviewer* :change-background #f(0.7 0.3 0.3))

(setq *tf* (instance ros::transform-listener :init))

(setq *robot* *atlas*)
(init-fullbody-ik-main)
(make-foot-convex)

(ros::advertise-service "/atlas_eus_ik" moveit_msgs::GetPositionIK #'ik-service-cb)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (x::window-main-one)
 (ros::spin-once))

#|
;; client sample
(ros::load-ros-manifest "hrpsys_gazebo_atlas")
(ros::load-ros-manifest "moveit_msgs")

(ros::roseus "foo")
(do-until-key 
(let ((req (instance moveit_msgs::GetPositionIKRequest :init))
      (D 0.2))
  (if (< (random 2.0) 1)
      (if (< (random 2.0) 1)
          (send req :ik_request :group_name "larm")
        (send req :ik_request :group_name "larm_torso"))
    (if (< (random 2.0) 1)
        (send req :ik_request :group_name "rarm")
      (send req :ik_request :group_name "rarm_torso")))
  (let ((c (make-coords)))
    (if (or (string= (send req :ik_request :group_name) "larm")
            (string= (send req :ik_request :group_name) "larm_torso"))
        (send c :rotate (deg2rad 90) :x)
      (progn
        (send c :rotate (deg2rad 180) :z)
        (send c :rotate (deg2rad 90) :x)))
      ;;(send c :rotate (deg2rad 90) :x))
    (let ((q (matrix2quaternion (send c :rot))))
  (send req :ik_request :pose_stamped (instance geometry_msgs::PoseStamped :init
                                                :header (instance std_msgs::Header :init
                                                                  :stamp (ros::time 0.0)
                                                                  :frame_id "/pelvis")
                                                :pose (instance geometry_msgs::Pose :init
                                                                :position (instance geometry_msgs::Point :init
                                                                                    :x (+ 0.8 (- (random D) (/ D 2.0)))
                                                                                    :y (- (random (* D 2)) (/ (* 2 D) 2.0))
                                                                                    :z (+ 0.3 (- (random D) (/ D 2.0))))
                                                                :orientation
                                                                (instance geometry_msgs::Quaternion :init
                                                                                       :x (elt q 1)
                                                                                       :y (elt q 2)
                                                                                       :z (elt q 3)
                                                                                       :w (elt q 0))
                                                                )))
  (let ((res (ros::service-call "/atlas_eus_ik" req)))
    (if (= (send res :error_code :val) moveit_msgs::MoveItErrorCodes::*SUCCESS*)
        (print (send res :solution :joint_state :position))
      (print "failed!")
      )))
)))
(exit)

|#

