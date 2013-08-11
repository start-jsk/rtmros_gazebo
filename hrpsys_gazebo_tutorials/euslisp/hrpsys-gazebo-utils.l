#-:jsk
(jsk)
#-:rbrain-basic
(rbrain)

(defun eus2urdf-for-gazebo (obj &optional name)
  (let* ((model-name (string-downcase (or name (send obj :name))))
	 (tmp-dir "/tmp"))
    ;; check model is named
    (unless (send obj :name)
      (warn "(send obj :name) is nil. set name to model.~%")
      (return-from eus2urdf-for-gazebo))
    ;; convert eus model to collada
    (convert-rbrain-model-to-collada obj :model-name model-name :output-directory tmp-dir)
    ;; call python script
    (unix:system (format nil "rosrun hrpsys_gazebo_tutorials eus2urdf_for_gazebo_pyscript.py ~a ~a/~a-robot.dae" model-name tmp-dir model-name))
    ))
(warn "~%(eus2urdf-for-gazebo (make-askul-gdh-cupboard) \"cupboard\")~%")

(defun add-room-to-gazebo (room)
  (dolist (obj (send room :objects))
    (unless (send obj :name)
      (send obj :name "no-name"))
    (eus2urdf-for-gazebo obj))
  )
(warn "(add-room-to-gazebo (room73b2))~%")
