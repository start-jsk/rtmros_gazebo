#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(load "package://hrpsys_gazebo_tutorials/euslisp/gazebo-plugin-utils")
(load "package://euslib/demo/s-noda/hrp2/my-util.l")


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
