(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")

(defvar *touchit-reached* 9)
(defvar *touchit-touched* 3)

(defun init-touch-it-without-server
  (&key (real t))
  (cond ((and real (not (and (boundp '*ri*) *ri*)))
         (atlas-init-ex :set-reset-pose nil :real t :view nil))
        ((not (and (boundp '*atlas*) *atlas*))
         (atlas-init-ex :set-reset-pose nil :real nil :view nil)
	 (send *atlas* :reset-manip-pose)))
  (setq *do-not-start-ik-server* t)
  (load "package://hrpsys_gazebo_atlas/euslisp/atlas-eus-ik.l")
  ;; (setq *do-not-start-touch-it* t)
  ;; (load "package://hrpsys_gazebo_atlas/euslisp/touch-it-server.l")
  (switch-end-coords :id 2)
  (when (and (boundp '*ri*) *ri*)
    (send *ri* :set-interpolation-mode ":linear"))
  (ros::ros-warn "[touch_it] init-touchit-without-server done with real ~A" real)
  )


(defun move-down-with-touch-it
  ()
  (send *atlas* :old-reset-manip-pose)
  (send *atlas* :rarm :move-end-rot 90 :x :local)
  (send *atlas* :rarm :move-end-pos #f(0 0 150) :world :rotation-axis :z :revert-if-fail t)
  (send *atlas* :rarm :move-end-rot -15 :y :local :revert-if-fail nil)
  (when (and (boundp '*irtviewer*) *irtviewer*)
    (send *irtviewer* :draw-objects))
  (when (and (boundp '*ri*) *ri*)
    (model2real :time 3000 :wait t))
  (my-wait)
  (touch-it-control (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 -300) :world)))


(defun move-lateral-with-touch-it
  ()
  (send *atlas* :old-reset-manip-pose)
  (send *atlas* :rarm :move-end-pos #f(0 0 -200))
  (when (and (boundp '*irtviewer*) *irtviewer*)
    (send *irtviewer* :draw-objects))
  (when (and (boundp '*ri*) *ri*)
    (model2real :time 3000 :wait t))
  (my-wait)
  (touch-it-control (send (send (send *atlas* :rarm :end-coords) :copy-worldcoords) :translate #f(0 0 300)))) 

(defun touch-it-control
  (tc &key (arm :rarm) (ik-type arm))
  (let ((i 0)
        (f-orig))
    
    (when (and (boundp '*ri*) *ri*)
      (setq f-orig (if (send *ri* :state :force-vector arm)
		       (send *ri* :state :absolute-force-vector arm)
		     #f(0 0 0))))

    (do-until-key
     (let ((touch-it-status
            (touch-it-control-proc tc :arm arm :ik-type ik-type :i i :f-orig f-orig)))
       (print touch-it-status)
       (incf i)
       (cond
        ((eq touch-it-status :arrived-at-goal)
         (return-from nil nil))
        ((eq touch-it-status :continue)
	 )
        ((eq touch-it-status :touched)
         (return-from nil t))
        ))
     ))
  (when (and (boundp '*ri*) *ri*)
    (real2model)
    (model2real :time 500))
  ) 

;; return value:
;;     :arrived-at-goal
;;     :continue
;;     :touched
;;     
(defun touch-it-control-proc
  (tc &key
      (arm :rarm)
      (i 0)
      (f-orig nil)
      (ik-type arm)
      (axis nil)
      (div 30)
      (thre 20))
  ;; calculate target
  (let (;;(div 30)
        (touch-flag :continue))
    (block :touch-it-loop
      ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; check count
      (when (> i div)
	(setq touch-flag :arrived-at-goal)
	(return-from :touch-it-loop nil))
       ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; check the force sensor value
      (let* (f
	     f-conv
	     dir
	     val)
	(if (and (boundp '*ri*) *ri*)
	    (setq f (v- 
		     (if (send *ri* :state :force-vector arm)
			 (send *ri* :state :absolute-force-vector arm)
		       #f(0 0 0))
		     f-orig))
	  (setq f (float-vector 0 0 0)))
	(setq f-conv (float-vector (elt f 0) (- (elt f 1)) (- (elt f 2))))
	(setq dir (normalize-vector
		   (v- (send tc :worldpos)
		       (send (send *atlas* arm :end-coords) :worldpos))))
	(setq val (abs (v. f-conv dir)))
	(ros::ros-warn "[touch_it] ~a force = ~a # touchitforce~%" arm f)
	(ros::ros-warn "[touch_it] ~a dir = ~a  val = ~a  thre = ~a # touchitforce~%" arm dir val thre)
	;; (warning-message 2 "[touch_it] ~a force = ~a # touchitforce~%" arm f)
	;; (warning-message 2 "[touch_it] ~a dir = ~a  val = ~a  thre = ~a # touchitforce~%" arm dir val thre)
	(setq *touchit-val* val)
	(when (> val thre)
	  (setq touch-flag :touched)
	  (return-from :touch-it-loop nil)))
       ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
      ;; solve ik and move the robot
      (let* ((r (/ (+ i 1.0) div))
	     (c (midcoords r
			   (send (send *atlas* arm :end-coords) :worldcoords)
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
			    (list (send *atlas* :torso k :child-link)
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
		     (send *atlas* arm :inverse-kinematics c :stop 100
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
	(if (and (boundp '*ri*) *ri*)
	    (progn
	      (ros::ros-warn "[touch_it] sending to real robot")
	      ;; (model2real :time 2000 :wait t))
	      (send *ri* :angle-vector (send *atlas* :angle-vector) 2000)
	      (unix:usleep (* 1500 1000)))
	  (progn
	    (ros::ros-warn "[touch_it] sending to dummy robot")
	    (unix:usleep (* 20 1000))))
	(when (and (boundp '*irtviewer*) *irtviewer*)
	  (send *irtviewer* :draw-objects))
	)
      )                                 ;touch-it-loop
    (when (and (boundp '*irtviewer*) *irtviewer*)
      (send *irtviewer* :draw-objects))
    touch-flag
    ))

