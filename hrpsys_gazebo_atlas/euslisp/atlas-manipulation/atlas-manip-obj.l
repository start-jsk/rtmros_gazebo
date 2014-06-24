(defun abs-axis (axis)
  (cond ((equal axis :-x) :x)
	((equal axis :-y) :y)
	((equal axis :-z) :z)
	(t axis)))

(defun set-ref-force
  (&key (x 0) (y 0) (z 0) (arm :arms) &allow-other-keys)
  (send *ri* :set-impedance-controller-param arm :ref-force (float-vector x y z)))

(defun set-ref-moment
  (&key (x 0) (y 0) (z 0) (arm :arms) &allow-other-keys)
  (send *ri* :set-impedance-controller-param arm :ref-moment (float-vector x y z)))

(defun axis-symbol->float-vector
  (tmp-axis)
  (case tmp-axis
    (:x (float-vector 1 0 0)) (:-x (float-vector -1 0 0))
    (:y (float-vector 0 1 0)) (:-y (float-vector 0 -1 0))
    (:z (float-vector 0 0 1)) (:-z (float-vector 0 0 -1))
    (t tmp-axis)))

(defun trans-world2fmc
  (c)
  (cond ((vectorp c)
	 (send (send *robot* :foot-midcoords) :rotate-vector c))
	((derivedp c coordinates)
	 (send (send c :copy-worldcoords) :transform (send *robot* :foot-midcoords) :world))
	))

(defun trans-fmc2world
  (c)
  (cond ((vectorp c)
	 (send (send *robot* :foot-midcoords) :inverse-rotate-vector c))
	((derivedp c coordinates)
	 (send (send c :copy-worldcoords) :transform (send (send *robot* :foot-midcoords) :inverse-transformation) :world))
	))

(defun reach-hand-until-touch
  (&key (axis :x) (f-thre 10.0) (arm :arms)
	(separate-lr nil) (opposite-lr nil) (pull-after-touch nil))

  (let* ((inc-dist 5)
	 (dt 500)
	 (larm-fin-p (if (equal arm :rarm) t nil))
	 (rarm-fin-p (if (equal arm :larm) t nil))
	 (arm-fin-p nil)
	 (force-vector-la-org
	  (concatenate float-vector 
		       (mapcar #'abs (concatenate cons (send *ri* :absolute-force-vector :larm)))))
	 (force-vector-ra-org
	  (concatenate float-vector
		       (mapcar #'abs (concatenate cons (send *ri* :absolute-force-vector :rarm))))))
    (if (or (equal arm :larm) (equal arm :rarm))
	(setq separate-lr t))
    ;; approach forward
    (set-ref-force (abs-axis axis) 0 :time 500)
    ;; (unix::usleep (* 500 1000))
    (block :reach-hand-until-touch-do-until-key-block
      (do-until-key
       ;; move hand forward
       ;; check IK fail
       (when (or (and (not separate-lr) (not arm-fin-p)) (and separate-lr (not larm-fin-p)))
	 (unless (send *robot* :larm :move-end-pos (scale inc-dist (trans-world2fmc (axis-symbol->float-vector axis))) :world)
	   (warning-message 4 "[reach-hand-until-touch] IK fail ~%")
	   (return)))
       (when (or (and (not separate-lr) (not arm-fin-p)) (and separate-lr (not rarm-fin-p)))
	 (unless (send *robot* :rarm :move-end-pos
		       (trans-world2fmc (scale inc-dist (scale (if opposite-lr -1 1)
							       (axis-symbol->float-vector axis)))) :world)
	   (warning-message 4 "[reach-hand-until-touch] IK fail ~%")
	   (return)))
       (model2real :time dt :wait nil)
       ;; check if force changed
       (let* ((force-vector-la 
	       (concatenate float-vector 
			    (mapcar #'abs (concatenate cons (v- (send *ri* :absolute-force-vector :larm) force-vector-la-org)))))
	      (force-vector-ra 
	       (concatenate float-vector
			    (mapcar #'abs (concatenate cons (v- (send *ri* :absolute-force-vector :rarm) force-vector-ra-org)))))
	      (force-la (abs (v. force-vector-la (axis-symbol->float-vector axis))))
	      (force-ra (abs (v. force-vector-ra (axis-symbol->float-vector axis))))
	      (force-a (* 0.5 (+ force-la force-ra))))
	 (if separate-lr
	     (progn
	       (when (and (not larm-fin-p) (> force-la f-thre))
		 (warning-message 3 "[reach-hand-until-touch] lforce[~a] > f-thre[~a]~%" force-la f-thre)
		 (setq larm-fin-p t)
		 )
	       (when (and (not rarm-fin-p) (> force-ra f-thre))
		 (warning-message 3 "[reach-hand-until-touch] rforce[~a] > f-thre[~a]~%" force-ra f-thre)
		 (setq rarm-fin-p t)
		 ))
	   (when (and (not arm-fin-p) (> force-a f-thre))
	     (warning-message 3 "[reach-hand-until-touch] force[~a] > f-thre[~a]~%" force-a f-thre)
	     (setq arm-fin-p t)
	     )))
       (when (or arm-fin-p (and larm-fin-p rarm-fin-p))
	 (return-from :reach-hand-until-touch-do-until-key-block nil))
       ;; wait
       (unix::usleep (round (* dt 1000))))
      (my-wait) ;; need for do-until-key bug
      )
    ;; pull slight distance after touch
    (when pull-after-touch
      (when (or (equal arm :arms) (equal arm :larm))
	(unless (send *robot* :larm :move-end-pos
		      (trans-world2fmc (scale (* (- pull-after-touch) inc-dist) 
					      (axis-symbol->float-vector axis))) :world)
	  (warning-message 4 "[reach-hand-until-touch] IK fail ~%")
	  (return-from reach-hand-until-touch)))
      (when (or (equal arm :arms) (equal arm :rarm))
	(unless (send *robot* :rarm :move-end-pos
		      (trans-world2fmc (scale (* (- pull-after-touch) inc-dist)
					      (scale (if opposite-lr -1 1)
						     (axis-symbol->float-vector axis)))) :world)
	  (warning-message 4 "[reach-hand-until-touch] IK fail ~%")
	  (return-from reach-hand-until-touch)))
      (model2real :time dt :wait t))
    ))


(defun reach-hand
  ()

  (let* ((pull-after-touch 8) (arm :arms))
    (warning-message 3 "[reach-hand] (reach-hand-until-touch :x) ~%")
    (reach-hand-until-touch :axis :x :f-thre 10 :arm arm
			    :pull-after-touch pull-after-touch)
    (warning-message 3 "[reach-hand] (reach-hand-until-touch :-y) ~%")
    (reach-hand-until-touch :axis :-y :f-thre 10 :arm arm
			    :separate-lr t :opposite-lr t :pull-after-touch pull-after-touch)
    (warning-message 3 "[reach-hand] adjustment of hand position ~%")
    (let* ((find-adj-trans-vec-larm
	    (trans-world2fmc (v+ (scale (* pull-after-touch 10) (axis-symbol->float-vector :x))
				 (scale (* pull-after-touch 10) (axis-symbol->float-vector :-y)))))
	   (find-adj-trans-vec-rarm
	    (trans-world2fmc (v+ (scale (* pull-after-touch 10) (axis-symbol->float-vector :x))
				 (scale (* pull-after-touch 10) (scale -1 (axis-symbol->float-vector :-y)))))))
      (when (or (equal arm :arms) (equal arm :larm))
	(unless (send *robot* :larm :move-end-pos find-adj-trans-vec-larm :world)
	  (warning-message 4 "[reach-hand] IK fail ~%") (return-from reach-hand)))
      (when (or (equal arm :arms) (equal arm :rarm))
	(unless (send *robot* :rarm :move-end-pos find-adj-trans-vec-rarm :world)
	  (warning-message 4 "[reach-hand] IK fail ~%") (return-from reach-hand))))
    (model2real :wait t :time 1000)
    (send *pickview* :draw-objects))
  )


