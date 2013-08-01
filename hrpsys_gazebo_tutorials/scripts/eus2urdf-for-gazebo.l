(defun eus2urdf-for-gazebo (obj &optional name)
  (let* ((model-name (string-downcase (or name (send obj :name))))
	 (tmp-dir "/tmp"))
    ;; convert eus model to collada
    (convert-rbrain-model-to-collada obj :model-name model-name :output-directory tmp-dir)
    ;; call python script
    (unix:system (format nil "rosrun hrpsys_gazebo_tutorials eus2urdf_for_gazebo_pyscript.py ~a ~a/~a-robot.dae" model-name tmp-dir model-name))
    )
  )
(warn "eus2urdf-for-gazebo (make-askul-gdh-cupboard) \"hoge\"~%")
