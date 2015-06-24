#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "move_base_msgs")

(ros::roseus "touch_it_action_node")

(load "package://drc_task_common/euslisp/robot-util.l")

(defun init-touchit-server
  ()
  (defvar *action-server* (instance ros::simple-action-server :init
                                "/touchit/act/exec" move_base_msgs::MoveBaseAction
                                :execute-cb 'touchit-action-cb
                                :preempt-cb 'touchit-preempt-cb))
  (ros::rate 1.0)
  (defvar *tfl* (instance ros::transform-listener :init))
  (unix::usleep (* 10000 1000))
  (generate-robot (unix::getenv "ROBOT") :generate-ri? t)
  (when (and (boundp '*irtviewer*) *irtviewer*)
    (send *irtviewer* :title "TouchIt!")
    (send *irtviewer* :change-background #f(0.2 0.2 0.8))
    (send *irtviewer* :draw-objects))
  (ros::ros-warn "[touch_it] touch_it_action_node started !~%")
  (defvar *touchit-goal-updated-p* t)
  (defvar *touchit-force-thre* 20)
  (defvar *touchit-val* 0)
  (defvar *touchit-ratio* 0)
  (defvar *touchit-ik-res* nil)
  (ros::subscribe "/touchit/thre" std_msgs::Float64 #'touchit-thre-cb)
  (do-until-key
   (when (boundp '*irtviewer*) (x::window-main-one))
   (send *action-server* :worker)
   (ros::spin-once)
   (ros::sleep))
  )
(defun touchit-control-proc
  (tc
   &key
   (arm :rarm)
   (i 0)
   (f-orig nil)
   (ik-type arm)
   (axis nil)
   (div 30)
   (thre 20))
  ;; calculate target
  (let ((touch-flag :continue))
    (block :touchit-loop
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; check count
      (when (> i div)
        (setq touch-flag :arrived-at-goal)
        (return-from :touchit-loop nil))
       ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; check the force sensor value
      (let* (f
             f-conv
             dir
             val)
        (setq f (v- (or (send *ri* :absolute-force-vector arm) #f(0 0 0)) f-orig))
        (setq f-conv (float-vector (elt f 0) (- (elt f 1)) (- (elt f 2))))
        (setq dir (normalize-vector
                   (v- (send tc :worldpos)
                       (send (send *robot* arm :end-coords) :worldpos))))
        (setq val (abs (v. f-conv dir)))
        (ros::ros-warn "[touch_it] ~a force = ~a # touchitforce~%" arm f)
        (ros::ros-warn "[touch_it] ~a dir = ~a  val = ~a  thre = ~a # touchitforce~%" arm dir val thre)
        (setq *touchit-val* val)
        (when (> val thre)
          (setq touch-flag :touched)
          (return-from :touchit-loop nil)))
       ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; solve ik and move the robot
      (let* ((r (/ (+ i 1.0) div))
             (c (midcoords r
                           (send (send *robot* arm :end-coords) :worldcoords)
                           tc)))

        ;; solve ik
        (ros::ros-warn "[touch_it] calculating pose <~a = ~a/~a> !~%" r i div)
        (setq *touchit-ratio* (* r 100))
        ;;(warning-message 1 "[touch_it] calculating pose <~a = ~a/~a> !~%" r i div)
        (setq *touchit-ik-res*
              (cond ((substringp "fullbody" (string ik-type))
                     (fullbody-ik-with-collision arm c))
                    ((substringp "torso" (string ik-type))
                     (apply
                      #'fullbody-ik-main
                      (list
                       :target-limb (list arm :lleg :rleg)
                       :target-coords (list c nil nil)
                       :additional-weight-list
                       (mapcar
                        #'(lambda (k gain)
                            (list (send *robot* :torso k :child-link)
                                  gain))
                        (list :waist-r :waist-p :waist-y)
                        '(0 0 1))
                       ;;(list 1e-3 1e-3 1e-3))
                       :root-link-virtual-joint-weight
                       (float-vector 0 0 0 0 0 0)
                       :min-vec (min
                                 (scale
                                  -1
                                  (map float-vector
                                       #'*
                                       (float-vector 0 0 0 0 0 0)
                                       #F(1000 1000 1000 200 200 200))))
                       :max-vec (map float-vector
                                     #'*
                                     (float-vector 0 0 0 0 0 0)
                                     #F(1000 1000 1000 200 200 200))
                       :collision? t
                       ;;:centroid-thre 200
                       ;;:target-centroid-pos nil
                       :thre '(1 1 1)
                       ;;:stop 100
                       :rotation-axis (list axis t t)))
                     ;; (torso-ik-with-collision arm c)
                     )
                    (t
                     (send *robot* arm :inverse-kinematics c :stop 100
                           :thre 2
                           :rthre (deg2rad 5)
                           :rotation-axis axis
                           ;; :dif-pos-ratio 1.00000 ;;
                           ;; :dif-rot-ratio 0.01000 ;;
                           ;; :manipulability-gain 0.00250  ;;
                           ;; :null-space nil ;;
                           ;; :additional-nspace-list nil ;;
                           ;; :avoid-nspace-gain 0 ;;
                           :revert-if-fail t))
                    ))
        (ros::ros-warn "[touch_it] ik result ~a !~%" *touchit-ik-res*)
        ;; send angle
        (ros::ros-warn "[touch_it] sending to real robot")
        (model2real :time 2000)
        (unix:usleep (* 1500 1000))
        (when (and (boundp '*irtviewer*) *irtviewer*) (send *irtviewer* :draw-objects))
        )
      )
    (when (and (boundp '*irtviewer*) *irtviewer*) (send *irtviewer* :draw-objects))
    touch-flag
    ))

(defun touchit-preempt-cb
  (server goal)
  ;; does not support distinghushing between goal, cancel everything always
  (setq *touchit-goal-updated-p* t)
  (real2model)
  (model2real :time 500)
  (send server :set-preempted)
  )

(defun touchit-action-cb
  (server goal)
  (let* ((pose-stamped (send goal :goal :target_pose))
         (pose (send pose-stamped :pose))
         (pos (send pose :position))
         (ori (send pose :orientation))
         (seq (send goal :goal :target_pose :header :seq))
         (vec (scale 1000.0 (float-vector (send pos :x) (send pos :y) (send pos :z))))
         (goal-coords (make-coords :pos vec))
         tc
         (ik-type
          (case (mod seq 10)
            (0 :larm)
            (1 :larm-torso)
            (2 :larm-fullbody)
            (3 :rarm)
            (4 :rarm-torso)
            (5 :rarm-fullbody)))
         touch-flag
         (axis
          (case (round (/ seq 10))
            (0 :z)
            (1 :x)
            (2 :y)
            (3 nil)
            (t t)))
         arm
         msg)
    (when (boundp '*irtviewer*) (x::window-main-one))
    (ros::ros-info "[touch_it] callback function was called.~%")
    ;; recieve arguments
    ;; (warning-message 4 "[touch_it] original pose timestamp is ~a!~%"
    ;;                  (send goal :goal :target_pose :header :stamp))
    (send goal :goal :target_pose :header :stamp (ros::time 0)) ;; This is a bug.
    ;; (warning-message 4 "[touch_it] overwrite pose timestamp is ~a!~%"
    ;;                  (send goal :goal :target_pose :header :stamp))
    (setq tc (send *tfl* :transform-pose "/BODY" pose-stamped))
    (cond ((substringp "larm" (string ik-type)) (setq arm :larm))
          ((substringp "rarm" (string ik-type)) (setq arm :rarm)))
    (unless tc (return-from touchit-action-cb nil))
    ;; get initial position only if the goal is new
    (if *touchit-goal-updated-p* (real2model))
    (when (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :draw-objects))
    (send* tc :rpy (car (rpy-angle (send (send *robot* arm :end-coords) :worldrot))))
    ;; control
    (ros::ros-info "[touch_it] pos = (~a, ~a, ~a)  ori = (~a, ~a, ~a, ~a)  sequence is ~a.~%"
                   (send pos :x) (send pos :y) (send pos :z) (send ori :w) (send ori :x) (send ori :y) (send ori :z) seq)
    (ros::ros-debug "[touch_it] tc is ~a~%" tc)
    ;; initialize global variables if needed
    (when *touchit-goal-updated-p*
      (setq *touchit-goal-updated-p* nil)
      (setq *touchit-force-reference* (or (send *ri* :absolute-force-vector arm) #f(0 0 0)))
      (setq *touchit-iteration-count* 0)
      (let* ((pos-crnt (send *robot* arm :end-coords :worldpos))
             (pos-dest (send tc :worldpos))
             (tmp-dist (norm (v- pos-crnt pos-dest))))
        (send *robot* :move-coords  (make-coords) (send *robot* :link "BODY"))
        (setq *touchit-iteration-div* (+ (/ tmp-dist 50) 1)))
      )
    (ros::ros-warn "[touch_it] arm=~A ik-type=~A axis=~A.~%" arm ik-type axis)
    (setq touch-flag
          (touchit-control-proc
           tc
           :arm arm
           ;; :ik-type ik-type
           :f-orig *touchit-force-reference*
           :i *touchit-iteration-count*
           :div *touchit-iteration-div*
           ;; :axis axis
           :thre *touchit-force-thre*))
    (incf *touchit-iteration-count*)
    ;; set result
    (setq msg (send server :result))
    ;; send result
    (ros::ros-warn "[touch_it] result is ~a .~%" msg)
    ;; Touched: Succeeded
    ;; Cancelled: Preemped
    ;; NotReached: Aboarted
    (cond ((eq touch-flag :continue)
           (ros::ros-info "[touch_it] continues to solve ik~%")
           ;; not sending goal yet, we will solve IK on the next iteratoin (:worker method calls :execution-cb)
           )
          ((eq touch-flag :arrived-at-goal)
           (ros::ros-info "[touch_it] finish ! reached !~%")
           (send server :set-aborted msg)
           (setq *touchit-goal-updated-p* t)
           )
          ((eq touch-flag :touched)
           (ros::ros-info "[touch_it] finish ! touched !~%")
           (send server :set-succeeded msg)
           (real2model)
           (model2real :time 500)
           (setq *touchit-goal-updated-p* t)
           ))
    (send server :publish-feedback
          (instance move_base_msgs::MoveBaseActionFeedback :init :feedback
                    (instance move_base_msgs::MoveBaseFeedback :init :base_position
                              (ros::coords->tf-pose-stamped (make-coords :pos (float-vector *touchit-val* *touchit-force-thre* *touchit-ratio*)
                                                                         :rpy (list (if *touchit-ik-res* 1 0) 0 0)) 0))))
    ))

(defun touchit-thre-cb
  (msg)
  (let* ((val (send msg :data)))
    (when (numberp val)
      (setq *touchit-force-thre* (max (min val 100) -1)))
    ))
