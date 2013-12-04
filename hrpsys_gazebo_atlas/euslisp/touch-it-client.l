#!/usr/bin/env roseus
;;;
;;; euslisp version of ~/ros/ros_pkg_tutorials/actionlib_tutorials/simple_action_servers/fibonacci_client.py
;;;
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")
;;(ros::load-ros-manifest "actionlib_tutorials")

(ros::roseus "touch_it_action_client_node")
;;(setq sys::*gc-hook* #'(lambda (a b) (format t ";; gc ~A ~A~%" a b)))

;; (ros::service-call "~set_logger_level"
;;                    (instance roscpp::SetLoggerLevelRequest
;;                              :init :logger "ros" :level "DEBUG"))


(defun touch-it-client
  (&optional (order (scale 0.001 (float-vector 479.364 -800.006 378.368)))
	     (ik-type 3)
	     (frame_id "/pelvis"))
  (let (goal res)
    ;; set request
    (if (not (boundp '*c*))
        (setq *c* (instance ros::simple-action-client :init
                            "/touchit/act/exec" move_base_msgs::MoveBaseAction)))
    (warning-message 2 "[touch_it_client] start waiting for server~%")
    (send *c* :wait-for-server)
    (warning-message 2 "[touch_it_client] finish waiting for server ~%")
    (setq goal (instance move_base_msgs::MoveBaseActionGoal :init))
    (send goal :goal :target_pose :header :frame_id frame_id)
    (send goal :goal :target_pose :header :stamp (ros::time 0.0))
    (send goal :goal :target_pose :pose :position :x (elt order 0))
    (send goal :goal :target_pose :pose :position :y (elt order 1))
    (send goal :goal :target_pose :pose :position :z (elt order 2))
    (send goal :goal :target_pose :pose :orientation :w 1.0)
    (send goal :goal :target_pose :header :seq ik-type)

    ;; send request
    (warning-message 2 "[touch_it_client] send goal to server ~%")
    (send *c* :send-goal goal)

    ;; wait
    (warning-message 2 "[touch_it_client] start waiting for result~%")

    ;; test cancel
    ;; (warning-message 2 "[touch_it_client] waitin 20 sec~%")
    ;; (unix:sleep 1)
    ;; (send *c* :cancel-all-goals)

    ;; check the result
    (send *c* :wait-for-result)
    ;; (warning-message 2 "[touch_it_client] result1: ~a ~%" (send *c* :get-result))
    (setq res (send *c* :get-state))
    ;; (warning-message 2 "[touch_it_client] result2: ~a ~%" res)
    res))

(warning-message 1 "~%")
(warning-message 1 "[touch_it_client] touch_it_action_client_node started !~%")

(setq res (touch-it-client (scale 0.001 (float-vector 479.364 0 378.368))))
(warning-message 1 "[touch_it_client] result: ~a~%" res)
(setq res (touch-it-client (scale 0.001 (float-vector 479.364 -800.006 378.368))))
(warning-message 1 "[touch_it_client] result: ~a~%" res)
(exit 0)

;; (let ((count 0))
;;   (do-until-key
;;    (warning-message 4 "[touch_it_client] loop: ~A/~A~%" count 1000)
;;    (incf count)
;;    (when (> count 1000)
;;      (exit 0))
;;    (setq res (touch-it-client))
;;    (warning-message 1 "[touch_it_client] result: ~a~%" res))
;;   (exit 0)
;;   )

