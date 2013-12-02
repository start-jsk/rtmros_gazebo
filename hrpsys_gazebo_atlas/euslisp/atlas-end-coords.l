
(require "atlas-model.l")

(defvar *robot* (atlas-with-hand))
(defvar *atlas* *robot*)

(defun set-end-coords
  (key coords
       &key
       (now-av (copy-object (send *robot* :angle-vector)))
       (now-c (copy-object (send *robot* :copy-worldcoords)))
       buf ret)
  (send *robot* :reset-manip-pose)
  (send *robot* :newcoords (make-coords))
  (let ((trans
	 (send (send *robot* key :end-coords)
	       :transformation
	       coords)))
    (send (send *robot* key :end-coords) :transform trans))
  (send *robot* :angle-vector now-av)
  (send *robot* :newcoords now-c)
  (send *robot* key :end-coords))

;; reset-manip-pose & pelvis 0-coords
(defvar *limb-key* '(:rarm :larm :rleg :lleg))
(defvar *switch-endcoords-collections*
  (list
   (list (make-coords :pos #f(474.777 -267.564 376.567) :rpy #f(7.748074e-06 -0.002922 -1.494227e-06)) (make-coords :pos #f(474.779 267.726 376.562) :rpy #f(-7.472436e-06 -0.00291 2.071643e-06)) (make-coords :pos #f(24.7508 -88.999 -794.516) :rpy #f(-3.582311e-09 -0.002932 1.221736e-06)) (make-coords :pos #f(24.7513 89.001 -794.515) :rpy #f(-3.582311e-09 -0.002932 1.221736e-06))) ;; default
   (list (make-coords :pos #f(535.86 -263.914 367.043) :rpy #f(-3.14159 -0.307211 1.57079)) (make-coords :pos #f(535.862 264.076 367.037) :rpy #f(-4.551172e-06 0.308817 1.57079)) (make-coords :pos #f(24.7508 -88.999 -794.516) :rpy #f(-3.582311e-09 -0.002932 1.221736e-06)) (make-coords :pos #f(24.7513 89.001 -794.515) :rpy #f(-3.582311e-09 -0.002932 1.221736e-06)))
   )
  )
(defvar *now-end-coords-id* 0)

(defun seq-end-coords
  (&key
   (now-av (copy-object (send *robot* :angle-vector)))
   (now-c (copy-object (send *robot* :copy-worldcoords)))
   buf
   ret
   )
  (send *robot* :reset-manip-pose)
  (send *robot* :newcoords (make-coords))
  (setq
   ret
   (cons
    'list
    (mapcar
     #'(lambda (k)
	 (setq buf (send *robot* k :end-coords :copy-worldcoords))
	 (list 'make-coords
	       :pos (send buf :worldpos)
	       :rpy (coerce (car (send buf :rpy-angle)) float-vector)))
     *limb-key*)))
  (send *robot* :angle-vector now-av)
  (send *robot* :newcoords now-c)
  ret)

(defun switch-end-coords
  (&key
   (id 0))
  (cond
   ((eq id *now-end-coords-id*)
    nil)
   ((< id (length *switch-endcoords-collections*))
    (setq *now-end-coords-id* id)
    (mapcar
     #'set-end-coords
     *limb-key*
     (nth id *switch-endcoords-collections*)))))



#|

(defun setup-end-coords ()
  (unless (boundp '*setup-end-coords-for-hku-flag*)
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
  (setq *setup-end-coords-for-hku-flag* t)
  )
