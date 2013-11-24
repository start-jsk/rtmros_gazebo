(defun joint-index-init nil
  (setq joint-name-list (list "back_bkz" "back_bky" "back_bkx" "neck_ry" "l_leg_hpz" "l_leg_hpx" "l_leg_hpy" "l_leg_kny" "l_leg_aky" "l_leg_akx" "r_leg_hpz" "r_leg_hpx" "r_leg_hpy" "r_leg_kny" "r_leg_aky" "r_leg_akx" "l_arm_shy" "l_arm_shx" "l_arm_ely" "l_arm_elx" "l_arm_wry" "l_arm_wrx" "r_arm_shy" "r_arm_shx" "r_arm_ely" "r_arm_elx" "r_arm_wry" "r_arm_wrx"))

  (setq i 0)
  (setq joint-index-list nil)
  (dolist (joint-name joint-name-list)
    (push (cons joint-name i) joint-index-list)
    (setq i (+ i 1))
    )
)