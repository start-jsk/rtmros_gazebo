(defun joint-index-init nil
  (setq atlas-joint-name-list
	(list
	 "back_bkz" "back_bky" "back_bkx" "neck_ry" "l_leg_hpz" "l_leg_hpx" "l_leg_hpy" "l_leg_kny" "l_leg_aky" "l_leg_akx" "r_leg_hpz" "r_leg_hpx" "r_leg_hpy" "r_leg_kny" "r_leg_aky" "r_leg_akx" "l_arm_shy" "l_arm_shx" "l_arm_ely" "l_arm_elx" "l_arm_wry" "l_arm_wrx" "r_arm_shy" "r_arm_shx" "r_arm_ely" "r_arm_elx" "r_arm_wry" "r_arm_wrx"

	"right_f2_j0" "right_f1_j0" "right_f1_j1" "right_f1_j2" "right_f3_j2" "right_f2_j2" "right_f3_j1" "right_f2_j1" "right_f0_j2" "right_f0_j1" "right_f0_j0" "right_f3_j0"

	"left_f1_j0" "left_f1_j1" "left_f1_j2" "left_f2_j1" "left_f2_j0" "left_f2_j2" "left_f0_j2" "left_f0_j1" "left_f0_j0" "left_f3_j2" "left_f3_j0" "left_f3_j1"
	)
	)
  (let (atlas-joint-indexes)
    (setq atlas-joint-indexes (make-list (length atlas-joint-name-list) :initial-element 0))
    (dotimes (i (length atlas-joint-indexes))
      (setf (elt atlas-joint-indexes i) i)
      )
    (setq ros-atlas-joint-index (coerce atlas-joint-indexes string))
    )


  (setq multisense-sl-joint-name-list
	(list
	 "hokuyo_joint" "pre_spindle_cal_x_joint" "pre_spindle_cal_y_joint" "pre_spindle_cal_z_joint" "pre_spindle_cal_roll_joint" "pre_spindle_cal_pitch_joint" "pre_spindle_cal_yaw_joint" "post_spindle_cal_x_joint" "post_spindle_cal_y_joint" "post_spindle_cal_z_joint" "post_spindle_cal_roll_joint" "post_spindle_cal_pitch_joint" "post_spindle_cal_yaw_joint"
	 )
	)
  (setq joint-name-list
	(append atlas-joint-name-list multisense-sl-joint-name-list)
	)
#|
  (setq joint-name-list
	(list
	 "back_bkz" "back_bky" "back_bkx" "neck_ry" "l_leg_hpz" "l_leg_hpx" "l_leg_hpy" "l_leg_kny" "l_leg_aky" "l_leg_akx" "r_leg_hpz" "r_leg_hpx" "r_leg_hpy" "r_leg_kny" "r_leg_aky" "r_leg_akx" "l_arm_shy" "l_arm_shx" "l_arm_ely" "l_arm_elx" "l_arm_wry" "l_arm_wrx" "r_arm_shy" "r_arm_shx" "r_arm_ely" "r_arm_elx" "r_arm_wry" "r_arm_wrx"

	"right_f2_j0" "right_f1_j0" "right_f1_j1" "right_f1_j2" "right_f3_j2" "right_f2_j2" "right_f3_j1" "right_f2_j1" "right_f0_j2" "right_f0_j1" "right_f0_j0" "right_f3_j0"

	"left_f1_j0" "left_f1_j1" "left_f1_j2" "left_f2_j1" "left_f2_j0" "left_f2_j2" "left_f0_j2" "left_f0_j1" "left_f0_j0" "left_f3_j2" "left_f3_j0" "left_f3_j1"

	 "hokuyo_joint" "pre_spindle_cal_x_joint" "pre_spindle_cal_y_joint" "pre_spindle_cal_z_joint" "pre_spindle_cal_roll_joint" "pre_spindle_cal_pitch_joint" "pre_spindle_cal_yaw_joint" "post_spindle_cal_x_joint" "post_spindle_cal_y_joint" "post_spindle_cal_z_joint" "post_spindle_cal_roll_joint" "post_spindle_cal_pitch_joint" "post_spindle_cal_yaw_joint"
	      ))

|#
  (setq i 0)
  (setq joint-index-list nil)
  (dolist (joint-name joint-name-list)
    (push (cons joint-name i) joint-index-list)
    (setq i (+ i 1))
    )
)

(defun get-joint-index (joint-name)
  (cdr (assoc joint-name joint-index-list :key #'car :test #'string=))
)
