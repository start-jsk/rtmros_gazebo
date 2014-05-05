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
 

    ;; apply eus pose to gazebo
 ;  (send *eusgazebo-server* :eus2gzb)


    (let* ((msg (instance gazebo_msgs::ModelState :init)))
      (send msg :model_name (send *top1* :get :gazebo-model-name))
      (send msg :pose (coords->pose (send *top1* :copy-worldcoords)))
      (send msg :twist :linear :x -1)
      (send msg :twist :angular :z 200) 
      (ros::publish "/gazebo/set_model_state" msg))

    (let* ((msg (instance gazebo_msgs::ModelState :init)))
      (send msg :model_name (send *top2* :get :gazebo-model-name))
      (send msg :pose (coords->pose (send *top2* :copy-worldcoords)))
      (send msg :twist :linear :x 1)
      (send msg :twist :angular :z 60) 
      (ros::publish "/gazebo/set_model_state" msg))
  
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
