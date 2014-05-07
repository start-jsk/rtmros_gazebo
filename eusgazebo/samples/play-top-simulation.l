(load "package://eusgazebo/euslisp/eusgazebo.l")


(defun init-top-simulation
  ()

  ;; make and view eus model
  (load "package://eusgazebo/samples/top-object.l")
  (setq *top1* (top))
  (setq *top2* (top))

  (objects (list *top2* *top1*))

  ;; overwrite mass property
  (dolist (top (list *top1* *top2*))
    (overwrite-mass-property top (float-vector 0 0 600) 10000 (list 1e10 1e10 1e9)))

  ;; generate eusgazebo instance
  (setq *eusgazebo-server* (instance eusgazebo :init))
  (send *eusgazebo-server* :add-model *top1*)
  (send *eusgazebo-server* :add-model *top2*)

  (unix::usleep (* 1000 1000)))


(defun play-top-simulation
  ()

  ;; start simulation
  (progn
    ;; pause simulation
    (send *eusgazebo-server* :pause-sim)
    ;; set eus model pose
    (send *top1* :newcoords (make-coords :pos #f(2000 0 1000) :rpy (list 0 0 0)))
    (send *top2* :newcoords (make-coords :pos #f(-2000 0 1000) :rpy (list 0 0 0)))

    (send *top1* :put :linear-vel (float-vector -1 0 0))
    (send *top1* :put :angular-vel (float-vector 0 0 200))
    (send *top2* :put :linear-vel (float-vector 1 0 0))
    (send *top2* :put :angular-vel (float-vector 0 0 60))

    (send *eusgazebo-server* :eus2gzb :set-vel t)
  
    ;; unpause simulation
    (send *eusgazebo-server* :unpause-sim)
    ;; view
    (dotimes (i 500)
      (send *eusgazebo-server* :gzb2eus)
      (send *irtviewer* :draw-objects)
      (x::window-main-one)
      (unix::usleep (* 10 1000)))
    ;;(send *eusgazebo-server* :gzb2eus-loop)
    ))
