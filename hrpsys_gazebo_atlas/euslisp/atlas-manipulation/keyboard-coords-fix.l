
(require "atlas-model.l")
(require "models/arrow-object.l")

(atlas-with-hand)
(setq *robot* *atlas*)

(defvar *viewer-update* t)
(sys:make-thread 3)

(defvar *rhand-arrow* (arrow))
(defvar *lhand-arrow* (arrow))

(defun mirror-coords
  (coords
   &key
   (plane #F(0 1 0))
   (rot-filter
    (map float-vector
	 #'(lambda (n) (cond ((zerop n) -1) (t 1)))
	 plane))
   (pos-filter (scale -1 rot-filter))
   )
  (make-coords
   :pos
   (map float-vector #'*
	pos-filter (send coords :worldpos))
   :rpy
   (map cons #'* rot-filter
	(car (rpy-angle (send coords :worldrot))))))

(defvar *right-hand-offset*
  (make-coords
   :rot
   (copy-object
    #2f((0.34202 -1.414503e-07 -0.939693)
	(-0.939692 2.591651e-07 -0.34202)
	(-2.643809e-07 1.0 9.436896e-16)))))
;    #2f((0.327076 0.048235 -0.943767)
;	(-0.898632 0.324861 -0.29483)
;	(0.292372 0.944531 0.149599)))))
;    #2f((0.327076 -0.099997 -0.939693)
;	(-0.898632 0.27474 -0.34202)
;	(0.292372 0.956305 9.324139e-16)))))
;    #2f((0.321394 -0.116978 -0.939693)
;	(-0.883022 0.321394 -0.34202)
;	(0.34202 0.939693 9.445569e-16)))))
(defvar *left-hand-offset*
  (mirror-coords *right-hand-offset*))

(objects (list *robot* *rhand-arrow* *lhand-arrow*))

(defun viewer-update
  nil
  (print 'start-viewer-update)
  (while *viewer-update*
    (x::window-main-one)
    (unix:usleep (round (* 1e+3 300))))
  (print 'stop-viewer-update)
  )

(defun keyboard-coords-fix
  (&key (mode :rpy))
  (setq *viewer-update* t)
  (sys:thread #'viewer-update)
  (while t
    ;;
    ;;
    (let* ((in (read-line)) sym deg)
      (cond
       ((zerop (length in)) nil)
       ((equal in "q")
	(setq *viewer-update* nil)
	(setq *right-hand-offset*
	      (make-coords
	       :rot
	       (m*
		(transpose
		 (send
		  (send *robot* :rarm
			:end-coords :worldcoords)
		  :worldrot))
		(send (send *rhand-arrow* :worldcoords)
		      :worldrot)
		)))
	(setq *left-hand-offset*
	      (mirror-coords *right-hand-offset*))
	(return-from nil nil))
       ((and (setq sym
		   (read-from-string (format nil ":~A" in)))
	     (find sym '(:x :y :z))
	     (setq in (subseq in 2))
	     (setq deg (if (zerop (length in)) 5
			 (read-from-string in)))
	     (numberp deg))
	(case mode
	      (:rpy
	       (send *rhand-arrow* :rotate (deg2rad deg) sym)
	       (send *viewer* :draw-objects))
	      (:move
	       (send *rhand-arrow* :translate
		     (scale deg
			    (cdr (assoc sym
					(list
					 (cons :x #F(1 0 0))
					 (cons :y #F(0 1 0))
					 (cons :z #F(0 0 1))))))
		     :world)
	       (send *viewer* :draw-objects))))))
    ;;
    ))

(defun hand-end-coords
  (&key
   (hand-key :rarm)
   (hand-offset
    (case hand-key
	  (:rarm *right-hand-offset*)
	  (:larm *left-hand-offset*))))
  (make-coords
   :pos
   (v+ (send hand-offset :worldpos)
       (send (send *robot* hand-key :end-coords :worldcoords)
	     :worldpos))
   :rot
   (m*
    (send (send *robot* hand-key :end-coords :worldcoords)
	  :worldrot)
    (send hand-offset :worldrot))))

(defun move-atlas-test
  (&key
   (angle-vector
    (copy-object (send *robot* :angle-vector)))
   (offset? t))
  (send *robot* :angle-vector angle-vector)
  (cond
   (offset?
    (send *rhand-arrow* :newcoords
	  (hand-end-coords :hand-key :rarm))
    (send *lhand-arrow* :newcoords
	  (hand-end-coords :hand-key :larm))))
  (send *viewer* :draw-objects)
  )

(defun gen-yaml-rotation
  (&key
   (arm :rarm))
  (let* ((a (hand-end-coords :hand-key arm))
	 (b (send *robot* arm :end-coords))
	 (c
	  (rotation-angle
	   (m* (transpose (send a :worldrot))
	       (send b :worldrot)))))
    (concatenate
     float-vector
     (cadr c)
     (list (* -1 (rad2deg (car c)))))))

(defmethod robot-model
  (:set-end-coords
   (&key
    (rhand-coords (hand-end-coords :hand-key :rarm))
    (lhand-coords (hand-end-coords :hand-key :larm)))
   (let* ((now-rhand (send self :rarm :end-coords))
	  (now-lhand (send self :larm :end-coords))
	  (rroot (send now-rhand :parent))
	  (lroot (send now-lhand :parent)))
     (send rroot :dissoc now-rhand)
     (send lroot :dissoc now-lhand)
     (setq
      rarm-end-coords
      (make-cascoords :init :link-list
		      :parent rroot
		      :coords rhand-coords
		      :name (send now-rhand :name)))
     (setq
      larm-end-coords
      (make-cascoords :init :link-list
		      :parent lroot
		      :coords lhand-coords
		      :name (send now-lhand :name)))
     (setq *right-hand-offset* (make-coords))
     (setq *left-hand-offset* (make-coords))
     (list rarm-end-coords larm-end-coords))))

(move-atlas-test)
;(send *robot* :set-end-coords)
#|

(send *robot*
      :fullbody-inverse-kinematics
      (list (make-coords :pos #F(1000 0 0))
	    (make-coords :pos #F(1000 0 0)))
      :move-target
      (mapcar
       #'(lambda (k)
	   (send *robot* k :end-coords))
       '(:rarm :larm))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list
		 (send *robot* k :end-coords :parent)))
       '(:rarm :larm))
      :rotation-axis (list t t)
      :translation-axis (list nil nil)
      :debug-view :no-message)

(send-all (send *robot* :joint-list) :max-joint-velocity 1.0)
(send *robot* :reset-pose)
(send *robot* :newcoords (make-coords))
(send *robot*
      :fullbody-inverse-kinematics
      (mapcar
       #'(lambda (k)
	   (let ((c (send *robot* k :end-coords :copy-worldcoords)))
	     (cond
	      ((find k '(:rarm :larm))
	       (make-coords
		:pos (v+ (send c :worldpos)
			 #F(400 0 500))))
	      (t c))))
       '(:rarm :larm :rleg :lleg))
      :move-target
      (mapcar
       #'(lambda (k)
	   (send *robot* k :end-coords))
       '(:rarm :larm :rleg :lleg))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list
		 (send *robot* k :end-coords :parent)))
       '(:rarm :larm :rleg :lleg))
      :root-link-virtual-joint-weight
      #F(0.01 0.0 0.1 0.001 0.001 0.0)
      :additional-weight-list
      (mapcar
       #'(lambda (k gain)
	   (list (send *robot* :torso k :child-link)
		 gain))
       (list :waist-r :waist-p :waist-y)
       (list 1e-3 1e-3 1e-3))
      :rotation-axis (list t t t t)
      :translation-axis (list :y :y t t)
      :debug-view :no-message)


(send *robot*
      :fullbody-inverse-kinematics
      (list (make-coords) (make-coords))
      :move-target
      (mapcar
       #'(lambda (k)
	   (send *robot* k :end-coords))
       '(:rarm :larm))
      :link-list
      (mapcar
       #'(lambda (k)
	   (send *robot* :link-list
		 (send *robot* k :end-coords :parent)))
       '(:rarm :larm))
      :translation-axis (list nil nil)
      :debug-view :no-message)