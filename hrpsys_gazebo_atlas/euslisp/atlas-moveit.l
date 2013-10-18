(require :atlas-interface
         "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require :pr2eus-moveit "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")

(defclass atlas-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod atlas-moveit-environment
  (:init
   (&key ((:robot rb) (atlas)))
   (send-super :init :robot rb
               :frame-id "pelvis"
               :multi-dof-joint-name "virtual_joint"
               :multi-dof-frame-id   "/world") ;; /world should be published
   )
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :target-link
                     (send self :search-link-from-name "r_hand"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "left_arm")
               (cons :target-link
                     (send self :search-link-from-name "l_hand"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :rleg
               (cons :group-name "right_leg")
               (cons :target-link
                     (send self :search-link-from-name "r_foot"))
               (cons :joint-list (append (send robot :rleg :joint-list)))
               )
         (list :lleg
               (cons :group-name "left_leg")
               (cons :target-link
                     (send self :search-link-from-name "l_foot"))
               (cons :joint-list (append (send robot :lleg :joint-list)))
               )
         (list :torso
               (cons :group-name "torso")
               (cons :target-link
                     (send self :search-link-from-name "utorso"))
               (cons :joint-list (append (send robot :torso :joint-list)))
               )
         (list :base
               (cons :group-name "base")
               (cons :target-link (send self :search-link-from-name "pelvis"))
               ;;(cons :joint-list ) virtual joint
               )
         (list :rarm-torso
               (cons :group-name "right_arm_torso")
               (cons :target-link
                     (send self :search-link-from-name "r_hand"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :rarm :joint-list)))
               )
         (list :larm-torso
               (cons :group-name "left_arm_torso")
               (cons :target-link
                     (send self :search-link-from-name "l_hand"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :larm :joint-list)))
               )

         )
   )
  )

#|
(setq *co* (instance collision-object-publisher :init))
(setq *me* (instance atlas-moveit-environment :init)) ;; call (atlas) inside it

(atlas)
(load "package://hrpsys_gazebo_atlas/models/drc-vehicle-simple.l")

(send *atlas* :fix-leg-to-coords (make-coords))
(send *drc-car* :translate (float-vector 0 1200 0))
(objects (list *atlas* *drc-car*))

(publish-collision-object *drc-car* *atlas* "pelvis")

(send *me* :sync-robot-model *atlas*)

;; collision aware ik
(send *me* :get-ik-for-pose
      (send *atlas* :larm :end-coords :copy-worldcoords)
      :larm :end-coords (list :larm :end-coords))

(send *me* :sync-robot-model *atlas* t)

(send *atlas* :reset-pose)
(send *me* :sync-robot-model *atlas*)
(send *me* :motion-plan :larm)

(send *atlas* :reset-pose)
(send *me* :sync-robot-model *atlas*)
(send *me* :motion-plan :rarm-torso)

|#