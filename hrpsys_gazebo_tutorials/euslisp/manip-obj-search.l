#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(load "package://hrpsys_gazebo_tutorials/euslisp/gazebo-plugin-utils")
(load "package://euslib/demo/s-noda/hrp2/my-util.l")



(defun in-range (x max min)
  (and (> x min) (< x max)))

(defun ss-square (x)
  (if (<= x 0) 0 (* x x)))

(defun range-square (x max min)
  (+ (ss-square (- x max)) (ss-square (- min x))))

(defun random-in-range (max min)
  (+ (random (- max min)) min)
  )

(defun get-random-idx-from-cum-ratio-list (crl)
  (let* ((random-num (random 1.0))
	 (ret (length crl)))
    (dotimes (i (length crl))
      (when (< random-num (elt crl i))
	(setq ret i)
	(return)))
    ret))


(defun test ()
  (let* ((learning-elenum 20)
	 (learning-cnt 100)
	 (total-frame 100)
	 (frame-dt 50)
	 (fx-max 20.0) (fx-min (- fx-max))
	 (fy-max 20.0) (fy-min (- fy-max))
	 (fz-max 20.0) (fz-min (- fz-max))
	 (mx-max 0.50) (mx-min (- mx-max))
	 (my-max 0.50) (my-min (- my-max))
	 (mz-max 0.50) (mz-min (- mz-max))
	 (fm-seq-list nil))
    ;; init list of gene
    (dotimes (i learning-elenum)
      (let* ((init-fm-seq))
	(dotimes (i total-frame)
	  (let* ((init-f (float-vector (random-in-range fx-max fx-min) (random-in-range fy-max fy-min) (random-in-range fz-max fz-min)))
		 (init-m (float-vector (random-in-range mx-max mx-min) (random-in-range my-max my-min) (random-in-range mz-max mz-min)))
		 (init-fm (list init-f init-m)))
	    (push-back init-fm init-fm-seq)
	    ))
	(push-back init-fm-seq fm-seq-list)
	))
    ;; loop of learning cycle
    (dotimes (i learning-cnt)
      (warning-message 2 "learning cnt cycle ~a/~a start.~%" (+ i 1) learning-cnt)
      (let* ((learning-eval-list nil)
	     (select-ratio-list nil)
	     (fm-seq-idx 0))
	;; loop of gene
	(dolist (fm-seq fm-seq-list)
	  (incf fm-seq-idx)
	  (warning-message 2 "  gene idx ~a/~a start. " fm-seq-idx (length fm-seq-list))
	  (let* ((learning-eval-value))
	    ;; loop of time
	    (reset-world)
	    (unix::usleep (* 10 1000))
	    (dolist (fm fm-seq)
	      ;; add force and moment
	      (let* ((f (elt fm 0)) (m (elt fm 1)))
		(add-force f m)
		(unix::usleep (* frame-dt 1000))))
	    (reset-world)
	  ;;; end of loop of time
	    ;; calc evaluation value of object pose
	    (let* ((x-max *inf*) (x-min -1)
		   (y-max 1) (y-min -1)
		   (z-max 2) (z-min 0)
		   (roll-max 30) (roll-min -30)
		   (pitch-max 30) (pitch-min -30)
		   (yaw-max 30) (yaw-min -30)
		   (rpy-w 0.05)
		   (x-dir-w 10))
	      (let* ((crnt-pose (get-pose))
		     (crnt-pos (scale 0.001 (send crnt-pose :pos)))
		     (crnt-rpy (reverse (mapcar #'rad2deg (car (rpy-angle (send crnt-pose :rot))))))
		     (crnt-x (elt crnt-pos 0))
		     (crnt-y (elt crnt-pos 1))
		     (crnt-z (elt crnt-pos 2))
		     (crnt-roll (elt crnt-rpy 0))
		     (crnt-pitch (elt crnt-rpy 1))
		     (crnt-yaw (elt crnt-rpy 2)))
		(setq learning-eval-value
		      (+
		       ;; in range evaluation
		       (- (+ (range-square crnt-x x-max x-min)
			     (range-square crnt-y y-max y-min)
			     (range-square crnt-z z-max z-min)))
		       (- (* rpy-w
			     (+ (range-square crnt-roll roll-max roll-min)
				(range-square crnt-pitch pitch-max pitch-min)
				(range-square crnt-yaw yaw-max yaw-min))))
		       ;; x direction evaluation
		       (* x-dir-w (ss-square crnt-x))
		       ))
		))
	    (warning-message 2 "  learning-eval-value: ~a~%" learning-eval-value)
	    (push-back learning-eval-value learning-eval-list)
	    ))
	;;; end of loop of gene
	(warning-message 3 "learning-eval-value: ~a~%" learning-eval-list)
	;; linear scaleing of eval
	(let* ((ls-a 1)
	       (ls-b (/ (- (apply #'max learning-eval-list) (* (apply #'min learning-eval-list) learning-elenum)) (- learning-elenum 1) ))
	       (ls-learning-eval-list (mapcar #'(lambda (x) (+ (* ls-a x) ls-b)) learning-eval-list))
	       (ls-learning-eval-sum (apply #'+ ls-learning-eval-list)))
	  (setq select-ratio-list (mapcar #'(lambda (x) (/ x ls-learning-eval-sum)) ls-learning-eval-list))
	  (warning-message 3 "select-ratio-list: ~a~%" select-ratio-list)
	  )
	;; genetic manipulation
	(let* ((select-cum-ratio-list select-ratio-list)
	       (fm-seq-list-next))
	  (dotimes (i (- (length select-ratio-list) 1))
	    (setf (elt select-cum-ratio-list (+ i 1)) (+ (elt select-cum-ratio-list i) (elt select-cum-ratio-list (+ i 1))))
	    )
	  (warning-message 3 "select-cum-ratio-list: ~a~%" select-cum-ratio-list)
	  (dotimes (i learning-elenum)
	    (cond
	     ((< (random 1.0) 0.5)
	      ;; crossing
	      (let* ((fm-seq1 (elt fm-seq-list (get-random-idx-from-cum-ratio-list select-cum-ratio-list)))
		     (fm-seq2 (elt fm-seq-list (get-random-idx-from-cum-ratio-list select-cum-ratio-list)))
		     (cross-idx (random-in-range 0 (length fm-seq1))))
		(push-back (append (subseq fm-seq1 0 cross-idx) (subseq fm-seq2 cross-idx (length fm-seq1))) fm-seq-list-next)))
	     (t
	      ;; copy
	      (push-back (elt fm-seq-list (get-random-idx-from-cum-ratio-list select-cum-ratio-list)) fm-seq-list-next))
	     )))
	))
      ;;; end of loop of learning cycle
    ))




(defun add-fm-one-dir (dir-vec fm-max)
  (reset-world)
  (unix::usleep (* 10 1000))
  (let* ((fm-max-vec (scale fm-max dir-vec))
	 (f (subseq fm-max-vec 0 3))
	 (m (subseq fm-max-vec 3 6))
	 before-pose after-pose diff-pose)
    (setq before-pose (get-pose))
    (add-force f m)
    (unix::usleep (* 500 1000))
    (setq after-pose (get-pose))
    (setq diff-pose (send before-pose :transformation after-pose))
    (warn "dir: ~a  fm-max: ~a  pos: ~a  rot: ~a~%" dir-vec fm-max (send diff-pose :pos) (mapcar #'rad2deg (car (rpy-angle (send diff-pose :rot))))))
  (reset-world))













;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; init

(defun init-manip-obj-search ()
  (init-gazebo-plugin)
  (setq *robot* (setup-model-for-hrp2 :no-view t)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; test

(defun test-all-dir-dual (&optional (limb-list (list :larm :rarm)) (grasp-pos-list (list #f(500 0 0) #f(-500 0 0))))
  (let* ((limb-fm-max-all-dir-list 
	  (mapcar #'(lambda (limb) (calc-limb-fm-max-all-dir limb)) limb-list)))
    (dolist (dir-vec (gen-sgnd-dir-vec-list 6))
      (let* ((fm-max (get-fm-max-one-dir dir-vec limb-fm-max-all-dir-list grasp-pos-list)))
	(add-fm-one-dir dir-vec fm-max)))))

(defun test-all-dir-single (&optional (limb :rarm))
  (dolist (dir-vec (gen-sgnd-dir-vec-list 6))
    (let* ((fm-max (norm (get-limb-fm-max-one-dir dir-vec limb))))
      (add-fm-one-dir dir-vec fm-max))))

(defun test-all-dir-const ()
  (dolist (dir-vec (gen-sgnd-dir-vec-list 6))
    (let* ((fm-max 50))
      (add-fm-one-dir dir-vec fm-max))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; fm-max

(defun get-fm-max-one-dir (dir-vec limb-fm-max-all-dir-list grasp-pos-list)
  (let* (fm-max
	 (min-angle-dir-vec pi))
    (dolist (limb-fm-max-0 (elt limb-fm-max-all-dir-list 0))
      (dolist (limb-fm-max-1 (elt limb-fm-max-all-dir-list 1))
	(let* ((limb-fm-max-list
		(list (cdr limb-fm-max-0) (cdr limb-fm-max-1)))
	       (limb-f-max-list (mapcar #'(lambda (fm) (subseq fm 0 3)) limb-fm-max-list))
	       (limb-m-max-list (mapcar #'(lambda (fm) (subseq fm 3 6)) limb-fm-max-list))
	       (f-max-vec
		(apply #'v+ limb-f-max-list))
	       (m-max-vec
		(reduce #'v+ (append limb-m-max-list (mapcar #'(lambda (gpos lfm) (v* (scale 0.001 gpos) lfm)) grasp-pos-list limb-f-max-list))))
	       (tmp-fm-max-vec (concatenate float-vector f-max-vec m-max-vec))
	       (tmp-min-angle-dir-vec (abs (acos (v. dir-vec (normalize-vector tmp-fm-max-vec))))))
	  (when (> min-angle-dir-vec tmp-min-angle-dir-vec)
	    (setq min-angle-dir-vec tmp-min-angle-dir-vec)
	    (setq fm-max (norm tmp-fm-max-vec))))))
    fm-max))


(defun calc-limb-fm-max-all-dir (limb)
  (let* (res)
    (dolist (dir-vec (gen-sgnd-dir-vec-list 6))
      (push (cons dir-vec (get-limb-fm-max-one-dir dir-vec limb)) res))
    res))

(defun get-limb-fm-max-one-dir (dir-vec limb)
  (let* ((robot *robot*)
	 (force-dir dir-vec)
	 (ext-force-res
	  (easy-calc-existable-ext-force
	   :force-dir force-dir
	   :move-target
	   (make-cascoords
	    :pos (send robot limb :end-coords :worldpos)
	    :rot (send robot limb :end-coords :worldrot)
	    :parent (send robot limb :end-coords :parent)
	    :limb-key limb))))
    (cdr (assoc :max ext-force-res))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; manip

(defun add-fm-one-dir (dir-vec fm-max)
  (reset-world)
  (unix::usleep (* 10 1000))
  (let* ((fm-max-vec (scale fm-max dir-vec))
	 (f (subseq fm-max-vec 0 3))
	 (m (subseq fm-max-vec 3 6))
	 before-pose after-pose diff-pose)
    (setq before-pose (get-pose))
    (add-force f m)
    (unix::usleep (* 500 1000))
    (setq after-pose (get-pose))
    (setq diff-pose (send before-pose :transformation after-pose))
    (warn "dir: ~a  fm-max: ~a  pos: ~a  rot: ~a~%" dir-vec fm-max (send diff-pose :pos) (mapcar #'rad2deg (car (rpy-angle (send diff-pose :rot))))))
  (reset-world))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; dir-vec

(defun gen-dir-vec-list (idx)
  (cdr (union (flatten (gen-dir-vec-list-rec (instantiate float-vector idx) 0)) nil :test #'equal)))

(defun gen-sgnd-dir-vec-list (idx)
  (cdr (union (flatten (gen-sgnd-dir-vec-list-rec (instantiate float-vector idx) 0)) nil :test #'equal)))

(defun gen-dir-vec-list-rec (vec idx)
  (when (> (length vec) idx)
    (let ((vec0 (copy-seq vec))
	  (vec1 (copy-seq vec)))
      (setf (elt vec0 idx) 0)
      (setf (elt vec1 idx) 1)
      (list (normalize-vector vec0) (normalize-vector vec1) (gen-dir-vec-list-rec vec0 (1+ idx)) (gen-dir-vec-list-rec vec1 (1+ idx))))))

(defun gen-sgnd-dir-vec-list-rec (vec idx)
  (when (> (length vec) idx)
    (let ((vec0 (copy-seq vec))
	  (vec1 (copy-seq vec))
	  (vec2 (copy-seq vec)))
      (setf (elt vec0 idx) 0)
      (setf (elt vec1 idx) 1)
      (setf (elt vec2 idx) -1)
      (list (normalize-vector vec0) (normalize-vector vec1) (normalize-vector vec2)
	    (gen-sgnd-dir-vec-list-rec vec0 (1+ idx)) (gen-sgnd-dir-vec-list-rec vec1 (1+ idx)) (gen-sgnd-dir-vec-list-rec vec2 (1+ idx))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; exe

(init-manip-obj-search)
