(require "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require "package://hrpsys_gazebo_atlas/euslisp/atlas-fullbody-ik.l")
(require "package://hrpsys_gazebo_atlas/models/drc_ladder.l")

(setq *ladder* (drc-ladder))
(atlas-init-ex :real nil)
(setq *robot* *atlas*)

(send *ladder* :rotate (deg2rad -90) :z)
(send *ladder* :translate #F(1180 0 0) :world)

(defmethod robot-model
  (:calc-static-balance-point
   (forces
    &optional (force-points
               (mapcar #'(lambda (tmp-arm)
                           (send (send self tmp-arm :end-coords) :worldpos))
                       '(:rarm :larm)))
    &key (ref-cog-height (aref (apply #'midpoint 0.5 (send self :legs :end-coords :worldpos)) 2)))
   (let* ((sbp (float-vector 0 0 ref-cog-height))
          (cog (send self :centroid))
          (mg (* 1e-6 (send self :weight nil) (elt *g-vec* 2))))
     (dotimes (idx 2)
       (let ((denom mg)
             (nume (* mg (elt cog idx))))
         (mapcar #'(lambda (f fp)
                     (setq nume
                           (+ nume
                              (- (* (- (elt fp 2) ref-cog-height) (elt f idx))
                                 (* (elt fp idx) (elt f 2))))
                           denom (- denom (elt f 2))))
                 forces force-points)
         (setf (elt sbp idx) (/ nume denom))
         ))
     sbp)))

(defun make-foot-convex
  (&key
   (target-limbs '(:rleg :lleg)))
  (setq *foot-convex*
	(make-prism
	 (quickhull
	  (mapcar
	   #'(lambda (v)
	       (concatenate float-vector
			    (subseq v 0 2)
			    (list
			     (+
			      -10
			      (apply
			       #'min
			       (mapcar
				#'(lambda (k)
				    (aref
				     (send *robot* k :end-coords :worldpos)
				     2))
				(remove-if
				 #'(lambda (k) (not (find k '(:rleg :lleg))))
				 target-limbs)))))))
	   (append
	    (mapcar
	     #'(lambda (hk)
		 (copy-object (send (my-end-coords hk) :worldpos)))
	     (remove-if
	      #'(lambda (k) (not (find k '(:rarm :larm))))
	      target-limbs))
	    (flatten
	     (mapcar
	      #'(lambda (k)
		  (send-all
		   (send
		    (send (my-end-coords k) :parent)
		    :faces)
		   :vertices))
	      (remove-if
	       #'(lambda (k) (not (find k '(:rleg :lleg))))
	       target-limbs)
	      )))))
	 5))
  (send *foot-convex* :set-color #F(0 0 1))
  (gl::transparent *foot-convex* 0.3))

(defvar *right-force* #f(0 0 0))
(defvar *left-force* #F(0 0 0))
(defun fix-centroid-object
  (&optional (centroid;; (send *robot* :centroid)) 
	      (send *robot* :calc-static-balance-point 
		    (list *right-force* *left-force*)))
	     (leg-h
	      (scale
	       0.5
	       (v+
		(send *robot* :rleg :end-coords :worldpos)
		(send *robot* :lleg :end-coords :worldpos)))))
  (if
      (and (boundp '*centroid-object*) *centroid-object*)
      (send
       *centroid-object*
       :newcoords
       (make-coords
	:pos
	(v+
	 #f(0.0 0.0 -200.0)
	 (concatenate
	  float-vector
	  (subseq centroid 0 2)
	  (list (+ 10 (aref leg-h 2)))))))))

(defun update-objects
  (&key
   (target-limbs '(:rleg :lleg)))
  (if (not (and (boundp '*centroid-object*) *centroid-object*))
      (make-centroid-object))
  (fix-centroid-object)
  (make-foot-convex :target-limbs target-limbs)
  (cond
   ((not (and (boundp '*viewer*) *viewer*))
    (objects
     (list *robot* *ladder* *centroid-object* *foot-convex*))
    (send *irtviewer* :change-background #f(1 1 1)))
   (t
    (send *irtviewer* :change-background #f(1 1 1))
    (send *irtviewer* :objects
	  (list *robot* *ladder* *centroid-object* *foot-convex*))))
  (send *viewer* :draw-objects))

(defun init-pose
  (&rest args)
  (send *robot* :reset-manip-pose)
  (send *robot* :newcoords (make-coords))
  (send *sandia_hand_left* :grasp-pose)
  (send *sandia_hand_right* :grasp-pose)
  ;;
  (send *robot* :rleg :crotch-y :joint-angle -15)
  (send *robot* :lleg :crotch-y :joint-angle 15)
  (send *robot* :lleg :inverse-kinematics
  	(send
  	 (send *robot* :lleg :end-coords :copy-worldcoords)
  	 :translate #F(0 10 0)
  	 :world)
  	:move-target (send *robot* :lleg :end-coords))
  (send *robot* :rleg :inverse-kinematics
  	(send
  	 (send *robot* :rleg :end-coords :copy-worldcoords)
  	 :translate #F(0 -10 0)
  	 :world)
  	:move-target (send *robot* :rleg :end-coords))
  ;;
  (send *robot* :newcoords
	(make-coords
	 :rpy (list 0 0 0)
	 :pos
	 (float-vector
	  -300 0
	  (+
	   0
	   (aref
	    (v- (send *robot* :worldpos)
		(send *robot* :rleg :end-coords :worldpos))
	    2)))))
  (send *viewer* :draw-objects)
  (update-objects)
  *robot*
  )

(setq *end-coords-collection* (make-hash-table))
(defun my-end-coords
  (key)
  (let (buf)
    (cond
     ((send *end-coords-collection* :get key)
      (send *end-coords-collection* :get key))
     ((find key '(:rarm :larm))
      (send *end-coords-collection* :enter key
	    (make-cascoords
	     :init :link-list
	     :name
	     (read-from-string
	      (format nil "~A-my-end-coords" key))
	     :parent (send *robot* key :end-coords :parent)
	     :coords
	     (make-coords
	      :pos
	      (copy-object
	       (send
		(send *robot* key :end-coords :parent)
		:worldpos))
	      ;; :rot
	      ;; (copy-object
	      ;;  (send
	      ;; 	(send *robot* key :end-coords :parent)
	      ;; 	:worldrot))
	      ))))
     (t (send *end-coords-collection* :enter key
	      (send *robot* key :end-coords))))))

(init-pose)
(objects (list *ladder* *robot*))

(setq *foot-print*
      (let ((now (float-vector 0 0 0))
	    (step (float-vector (/ 305 (tan (deg2rad 60))) 0 305))
	    (cnt 9))
	(mapcar
	 #'(lambda ()
	     (setq now (v+ now step))
	     (decf cnt)
	     (mapcar
	      #'(lambda (k)
		  (make-coords
		   :pos
		   (v+ now
		       (map float-vector
			    #'*
			    #F(0 1.0 0)
			    (send (my-end-coords k) :worldpos)))
		   :rot
		   (copy-object (send (my-end-coords k) :worldrot))))
	      '(:rarm :larm :rleg :lleg)))
	 (make-list cnt))))
(send-all (flatten *foot-print*) :draw-on :flush t)

(defun static-limb-move
  (&key
   (move-limb :rleg)
   (all-limbs '(:rarm :larm :rleg :lleg))
   (fix-limbs (remove move-limb all-limbs))
   (move #F(0 0 0))
   ;;
   (cog-gain 1.5)
   (leg-weight 10)
   (centroid-weight
    (normalize-vector
     (coerce
      (mapcar
       #'(lambda (k)
	   (if (find k '(:rleg :lleg))
	       leg-weight 1))
       fix-limbs)
      float-vector)))
   (centroid-thre #'check-centroid-with-convex)
   (additional-weight-list
    (mapcar
     #'(lambda (k gain)
	 (list (send *robot* :torso k :child-link)
	       gain))
     (list :waist-r :waist-p :waist-y)
     (list 1e-1 1e-1 1e-1)))
   (stop 100)
   ;;
   ;;(right-force #F(0 0 0))
   ;;(left-force #F(0 0 0))
   (centroid-offset-func
    '(lambda ()
       (x::window-main-one)
       (send *robot*
	     :calc-static-balance-point
	     (list *right-force* *left-force*))))
   (centroid-offset #F(0 0 0))
   (target-centroid-pos
    (v+
     centroid-offset
     (case move-limb
	   (:arms
	    (scale
	     0.5
	     (apply
	      #'v+
	      (mapcar
	       #'(lambda (k)
		   (send (my-end-coords k) :worldpos))
	       '(:rleg :lleg)))))
	   (t
	    (reduce #'v+
		    (map cons
			 #'scale
			 centroid-weight
			 (send-all
			  (mapcar #'my-end-coords fix-limbs)
			  :worldpos)))))))
   ;;
   (other-end-coords)
   (other-link-list
    (mapcar #'(lambda (mt)
		(send *robot* :link-list (send mt :parent)))
	    other-end-coords))
   (other-target-coords
    (send-all other-end-coords :copy-worldcoords))
   (other-translation-axis
    (make-list (length other-end-coords) :initial-element t))
   (other-rotation-axis
    (make-list (length other-end-coords) :initial-element t))
   (other-thre
    (make-list (length other-end-coords) :initial-element 5))
   (other-rthre
    (make-list (length other-end-coords) :initial-element (deg2rad 5)))
   )
  (let* ((mt (mapcar #'my-end-coords all-limbs))
	 (ll (mapcar
	      #'(lambda (m) (send *robot* :link-list (send m :parent)))
	      mt))
	 buf ret)
    (setq
     ret
     (send *robot* :fullbody-inverse-kinematics
	   (append
	    (mapcar
	     #'(lambda (mt k)
		 (setq buf (send mt :copy-worldcoords))
		 (cond
		  ((eq k move-limb)
		   (send buf :translate move :world))
		  (t t))
		 buf)
	     mt all-limbs)
	    other-target-coords)
	   :move-target (append mt other-end-coords)
	   :cog-gain cog-gain
	   :link-list (append ll other-link-list)
	   :debug-view :no-message
	   :min #F(-1000 -1000 -1000 -200 -200 -200)
	   :max #F(1000 1000 1000 200 200 200)
	   :translation-axis
	   (append
	    (make-list (length all-limbs) :initial-element t)
	    other-translation-axis)
	   :rotation-axis
	   (append
	    (make-list (length all-limbs) :initial-element t)
	    ;;'(:y :y t t)
	    other-rotation-axis)
	   :thre
	   (append
	    (make-list (length all-limbs) :initial-element 10)
	    other-thre)
	   :rthre
	   (append
	    (make-list (length all-limbs) :initial-element (deg2rad 5))
	    other-rthre)
	   :additional-weight-list additional-weight-list
	   :centroid-thre centroid-thre
	   ;;:centroid-offset-func
	   ;;#'(lambda nil
	   ;;(x::window-main-one)
	   ;;(send *robot* :centroid))
	   :centroid-offset-func centroid-offset-func
	   :target-centroid-pos
	   target-centroid-pos
	   ;; :obstacles (list (send *ladder* :link_lk))
	   ;; :collision-avoidance-link-pair
	   ;; (apply
	   ;;  #'append
	   ;;  (mapcar
	   ;;   #'(lambda (k)
	   ;; 	(mapcar
	   ;; 	 #'(lambda (l)
	   ;; 	     (list l (send *ladder* :link_lk)))
	   ;; 	 (send *robot* k :links)))
	   ;;   '(:rarm :larm)))
	   :root-link-virtual-joint-weight
	   #f(0.1 0.1 0.5 0.1 0.5 0.1)
	   :stop stop
	   ))
    (if (vectorp ret)
	(update-objects :target-limbs fix-limbs))
    ret))

(defun on-ladder
  (&rest
   args
   &key
   (move-limb :rleg)
   (all-limbs '(:rleg :lleg))
   (now-c (send (my-end-coords move-limb) :copy-worldcoords))
   target-c
   &allow-other-keys)
  (setq
   target-c
   (sort
    (mapcar
     #'(lambda (l)
	 (nth
	  (case move-limb
		(:rarm 0) (:larm 1) (:rleg 2) (:lleg 3))
	  l))
     *foot-print*)
    #'(lambda (a b)
	(< (norm2
	    (concatenate
	     float-vector
	     (scale 1e-3 (send a :difference-position now-c))
	     (send a :difference-rotation now-c)))
	   (norm2
	    (concatenate
	     float-vector
	     (scale 1e-3 (send b :difference-position now-c))
	     (send b :difference-rotation now-c)))))))
  (apply
   #'static-limb-move
   (append
    (list
     :move-limb move-limb
     :all-limbs all-limbs
     :move (v- (send (car target-c) :worldpos)
	       (send now-c :worldpos)))
    args)))

(defun torso-move
  (&rest
   args
   &key
   (move #F(0 0 10))
   &allow-other-keys)
  (apply
   #'static-limb-move
   (append
    (list
     :move-limb :arms
     :other-end-coords
     (list (send *robot* :torso :end-coords))
     :other-target-coords
     (list
      (send (send *robot* :torso :end-coords :copy-worldcoords)
	    :translate move :world))
     :other-link-list
     (list
      (send *robot* :link-list (send *robot* :torso :end-coords :parent)))
     :other-thre (list (min 1 (* (norm move) 0.9)))
     :other-rotation-axis (list nil)
     :other-translation-axis (list :y))
    args))
  )

(send *robot* :rleg :knee-p :min-angle 10)
(send *robot* :lleg :knee-p :min-angle 10)
(defun demo-static-climb-ladder
  (&key
   (cnt 9))
  (init-pose)
  (do-until-key
   (if (minusp (decf cnt)) (return-from nil t))
   (if
       (not
	(and
	 (static-leg-move :move-key :both)
	 (static-leg-move :move-key :rleg)
	 (static-leg-move :move-key :rleg :move #F(0 0 100))
	 (static-leg-move :move-key :rleg :move #F(0 0 100))
	 (static-leg-move :move-key :rleg :move #F(0 0 100))
	 (static-leg-move :move-key :rleg :move #F(0 0 50))
	 (or
	  t
	  (not (eq cnt 0))
	  (static-leg-move :move-key :rleg :move #F(0 0 30)))
	 (static-leg-move :move-key :rleg :move #F(100 0 0))
	 (static-leg-move :move-key :rleg :move #F(100 0 0))
	 (or
	  (not (eq cnt 0))
	  (static-leg-move :move-key :rleg :move #F(50 0 0)))
	 (on-ladder :move-key :rleg)
	 (static-leg-move :move-key :both)
	 (static-leg-move :move-key :lleg)
	 (static-leg-move :move-key :lleg :move #F(0 0 100))
	 (static-leg-move :move-key :lleg :move #F(0 0 100))
	 (static-leg-move :move-key :lleg :move #F(0 0 100))
	 (static-leg-move :move-key :lleg :move #F(0 0 50))
	 (or
	  t
	  (not (eq cnt 0))
	  (static-leg-move :move-key :lleg :move #F(0 0 30)))
	 (static-leg-move :move-key :lleg :move #F(100 0 0))
	 (static-leg-move :move-key :lleg :move #F(100 0 0))
	 (or
	  (not (eq cnt 0))
	  (static-leg-move :move-key :lleg :move #F(50 0 0)))
	 (on-ladder :move-key :lleg))
	)
       (return-from nil nil)))
  )

(defun gen-rsd
  (tag)
  (list (cons :tag tag)
	(cons :angle-vector
	      (copy-object (send *robot* :angle-vector)))
	(cons :worldcoords
	      (copy-object (send *robot* :worldcoords)))))

(defun play-rsd
  (rsd)
  (format t "[play-rsd] ~A~%" (cdr (assoc :tag rsd)))
  (send *robot* :angle-vector
	(copy-object (cdr (assoc :angle-vector rsd))))
  (send *robot* :newcoords
	(copy-object (cdr (assoc :worldcoords rsd))))
  (send *viewer* :draw-objects)
  )

(setq *odom* nil)
(ros::subscribe "/atlas/odom" geometry_msgs::PoseStamped
  #'(lambda (msg)
      (setq *odom* (ros::tf-pose-stamped->coords msg))))

(defvar *angle-vector-collection*)
(defun m2r
  nil
  (push (copy-object (send *robot* :angle-vector)) *angle-vector-collection*)
  (model2real :sleep-time 5000))

(defun r2m
  (&key
   (support-leg '(:rleg :lleg))); :lleg)))
  (real2model)
  (ros::spin-once) (ros::sleep)
  (ros::spin-once) (ros::sleep)
  (ros::spin-once) (ros::sleep)
  ;;(send *robot* :newcoords (send *odom* :copy-worldcoords))
  ;;(send *robot* :rotate (deg2rad 180) :z)
  ;; (mapcar
  ;;  #'(lambda (k)
  ;;      (send *robot* k :inverse-kinematics
  ;; 	     (make-coords)
  ;; 	     :translation-axis t
  ;; 	     :debug-view :no-message))
  ;;  support-leg)
  (send *robot* :fix-leg-to-coords (make-coords) (car support-leg))
  (update-objects))

;; #f(0.2728 25.0957 0.200876 68.3812 95.4757 17.3559 -9.84019 -38.041 120.288 -68.4782 96.3951 -16.5412 8.46219 -37.1436 118.89 56.871 -1.69028 -29.4271 1.67763 -21.6498 -0.178443 56.8317 1.97898 -29.4029 -1.9736 -21.6338 0.1912 0.0)

;; #f(-0.006031 21.5731 -0.00371 66.8495 95.763 -4.5047 3.588 -51.1387 94.5435 -66.8393 95.7746 4.50817 -3.58026 -51.1616 94.544 68.3758 0.005316 -53.0474 -0.005879 -21.0097 -0.000292 68.3762 0.005316 -53.0477 -0.005879 -21.0098 -0.000292 0.0)

#|

(init-pose)
(send *robot* :translate #F(50 0 0) :world)
(defvar *rsd*)
(defvar *cen*)
(setq *rsd* nil)
(setq *cen* nil)
(on-ladder :move-limb :lleg
	   :all-limbs '(:rleg :lleg)
	   :centroid-thre 100 :cog-gain 0.5)
(push (copy-object (send *robot* :centroid)) *cen*)
(push (gen-rsd :reach) *rsd*)
(static-limb-move :move-limb nil
		  :all-limbs '(:rleg :lleg)
		  :centroid-thre 100)
(static-limb-move :move-limb nil
		  :all-limbs '(:rleg :lleg)
		  :centroid-thre 100
		  :centroid-offset #F(10 10 0))
(static-limb-move :move-limb nil
		  :all-limbs '(:rleg :lleg)
		  :centroid-thre 100
		  :centroid-offset #F(10 10 0))
(send *robot* :torso :waist-p :joint-angle 100)
(push (copy-object (send *robot* :centroid)) *cen*)
(push (gen-rsd :balance) *rsd*)
(format t "h = ~A~%" (aref (send *robot* :centroid) 2))
(format t "d = ~A~%" (norm (apply #'v- *cen*)))
(format t "l = ~A~%" (norm (subseq
			    (send *robot* :rleg :end-coords :worldpos)
			    0 2)))
(format t "tt < ~A~%"
	(/ (/ (* (aref (send *robot* :centroid) 2)
		 (norm (apply #'v- *cen*)))
	      9800.0)
	   (norm (subseq
		  (v- (send *robot* :rleg :end-coords :worldpos)
		      (car *cen*))
		  0 2))))
(on-ladder :move-limb :rleg
	   :all-limbs '(:rleg :lleg)
	   :centroid-thre 10 :cog-gain 1.0)
(push  (list  (cons :tag :land) (cons :angle-vector (copy-object (send *robot* :angle-vector))) (cons :worldcoords (copy-object (send *robot* :worldcoords)))) *rsd*)
(dump-loadable-structure "dynamic-ladder-climb-data.l" *rsd*)

(init-pose)
(on-ladder :move-limb :larm :centroid-thre 100 :cog-gain 0.5)
(on-ladder :move-limb :rarm :centroid-thre 100 :cog-gain 0.5)
(static-limb-move
 :move-limb :arms
 :centroid-thre 50)

(do-until-key
 (torso-move :centroid-thre 1000
	     :cog-gain 0.05
	     :move-limb :arms
	     :move #F(0 0 10))
 )


(static-limb-move
 :move-limb :lleg
 :leg-weight 5
 :centroid-thre 100
 :move #F(0 0 10))

(on-ladder
 :move-limb :lleg
 :leg-weight 5
 :centroid-thre 100)

(static-limb-move
 :move-limb :arms
 :leg-weight 5
 :centroid-thre 100
 :move #F(0 0 0)
 )

(static-limb-move :move-limb :arms)

(send *robot* :fullbody-inverse-kinematics
      (list (make-coords) (make-coords))
      :move-target
      (mapcar
       #'my-end-coords
       '(:rarm :larm))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list (send *robot* k :end-coords :parent)))
       '(:rarm :larm))
      :translation-axis '(nil nil))

(send *robot*
      :fullbody-inverse-kinematics
      (list
       (send (send *robot* :rarm :end-coords :copy-worldcoords)
	     :translate #f(300 0 0))
       (send (send *robot* :larm :end-coords :copy-worldcoords)
	     :translate #f(300 0 0))
       (send (send *robot* :rleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       (send (send *robot* :lleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       )
      :move-target
      (mapcar
       #'(lambda (k) (send *robot* k :end-coords))
       '(:rarm :larm :rleg :lleg))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list (send *robot* k :end-coords :parent)))
       '(:rarm :larm :rleg :lleg))
      :debug-view :no-message))

(send *robot*
      :fullbody-inverse-kinematics
      (list
       (send (send *robot* :rarm :end-coords :copy-worldcoords)
	     :rotate (deg2rad -90) :x)
       (send (send *robot* :larm :end-coords :copy-worldcoords)
	     :rotate (deg2rad 90) :x)
       (send (send *robot* :rleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       (send (send *robot* :lleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       )
      :move-target
      (mapcar
       #'(lambda (k) (send *robot* k :end-coords))
       '(:rarm :larm :rleg :lleg))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list (send *robot* k :end-coords :parent)))
       '(:rarm :larm :rleg :lleg))
      :debug-view :no-message))

(send *robot*
      :fullbody-inverse-kinematics
      (list
       (send (send *robot* :rarm :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       (send (send *robot* :larm :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       (send (send *robot* :rleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       (send (send *robot* :lleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       )
      :target-centroid-pos
      (copy-object (send *robot* :rleg :end-coords :worldpos))
      :move-target
      (mapcar
       #'(lambda (k) (send *robot* k :end-coords))
       '(:rarm :larm :rleg :lleg))
      :centroid-thre 50
      :rthre (list (deg2rad 10) (deg2rad 10) (deg2rad 5) (deg2rad 5))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list (send *robot* k :end-coords :parent)))
       '(:rarm :larm :rleg :lleg))
      :debug-view :no-message))


(send *robot*
      :fullbody-inverse-kinematics
      (list
       (send (send *robot* :rarm :end-coords :copy-worldcoords)
	     :translate #f(50 0 0))
       (send (send *robot* :larm :end-coords :copy-worldcoords)
	     :translate #f(50 0 0))
       (send (send *robot* :rleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       (send (send *robot* :lleg :end-coords :copy-worldcoords)
	     :translate #f(0 0 0))
       )
      :centroid-thre 100
      :target-centroid-pos
      (copy-object (send *robot* :lleg :end-coords :worldpos))
      :move-target
      (mapcar
       #'(lambda (k) (send *robot* k :end-coords))
       '(:rarm :larm :rleg :lleg))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list (send *robot* k :end-coords :parent)))
       '(:rarm :larm :rleg :lleg))
      :stop 100
      :debug-view :no-message))

(atlas-init-ex :set-reset-pose nil)