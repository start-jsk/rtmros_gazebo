;;
;;
;;
(defclass top-object
  :super cascaded-link
  :slots (sensors
         ))
(defmethod top-object
  (:init
   (&rest args &key (name "top") (pos (float-vector 0 0 0)) (rot (unit-matrix 3)) &allow-other-keys)
   (let (c bc
         blink0
                 )
     (send-super* :init :name name args)

     ;; definition of link

     ;; definition of :top-bodyset2
     (setq bc (list
       (instance faceset :init :faces (list
         (instance face :init :vertices (list (float-vector 0 0 0) (float-vector 1000 1000 1000) (float-vector 1000 -1000 1000)))
         (instance face :init :vertices (list (float-vector 0 0 0) (float-vector -1000 1000 1000) (float-vector 1000 1000 1000)))
         (instance face :init :vertices (list (float-vector 0 0 0) (float-vector -1000 -1000 1000) (float-vector -1000 1000 1000) ))
         (instance face :init :vertices (list (float-vector 0 0 0) (float-vector 1000 -1000 1000) (float-vector -1000 -1000 1000)))
         (instance face :init :vertices (list (float-vector 1000 1000 1000) (float-vector -1000 1000 1000) (float-vector -1000 -1000 1000) (float-vector 1000 -1000.0 1000.0)))
       ))
       ))
     (dolist (b (cdr bc)) (send (car bc) :assoc b))
     (send (elt bc 0) :set-color (float-vector 0.8 0.1 0.1))
     (setq blink0 (instance bodyset-link :init (make-cascoords) :bodies bc :name :top-bodyset2 :weight 2000 :centroid (float-vector 0.0 0.0 0.0) :inertia-tensor #2f((1.0 0.0 0.0) (0.0 1.0 0.0) (0.0 0.0 1.0))))

     ;; definition of assoc
     (send self :assoc blink0)

     ;; definition of end-coords

     ;; definition of joint


     ;; init-ending 
     (setq links (list blink0))
     (setq joint-list (list))
     (send self :init-ending)
     (send self :move-to (make-coords :pos pos :rot rot))
     (send-all links :worldcoords)

     self))
  (:cameras (&rest args)
   (forward-message-to-all (list) args))

  (:handle (&rest args) (forward-message-to-all (list ) args))
  (:attention (&rest args) (forward-message-to-all (list ) args))
  (:button (&rest args) (forward-message-to-all (list ) args))
  )

(defun top (&rest args) (instance* top-object :init args))
;; (format *error-output* "(instance top-object :init) for generating model~%")
