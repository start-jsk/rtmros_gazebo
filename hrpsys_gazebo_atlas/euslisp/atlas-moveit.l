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
               :multi-dof-frame-id   "/world") ;;
   )
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "rarm")
               (cons :target-link
                     (send self :search-link-from-name "r_hand"))
               (cons :joint-list (send robot :rarm :joint-list))
               )
         (list :larm
               (cons :group-name "larm")
               (cons :target-link
                     (send self :search-link-from-name "l_hand"))
               (cons :joint-list (send robot :larm :joint-list))
               )
         (list :rleg
               (cons :group-name "rleg")
               (cons :target-link
                     (send self :search-link-from-name "r_foot"))
               (cons :joint-list (append (send robot :rleg :joint-list)))
               )
         (list :lleg
               (cons :group-name "lleg")
               (cons :target-link
                     (send self :search-link-from-name "l_foot"))
               (cons :joint-list (append (send robot :lleg :joint-list)))
               )
         (list :torso
               (cons :group-name "torso")
               ;;(cons :target-link
               ;;(send self :search-link-from-name "l_foot"))
               (cons :joint-list (append (send robot :torso :joint-list)))
               )
         (list :base
               (cons :group-name "base")
               (cons :target-link (send self :search-link-from-name "pelvis"))
               ;;(cons :joint-list ) virtual joint
               )
         (list :rarm-torso
               (cons :group-name "rarm_torso")
               (cons :target-link
                     (send self :search-link-from-name "r_hand"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :rarm :joint-list)))
               )
         (list :larm-torso
               (cons :group-name "larm_torso")
               (cons :target-link
                     (send self :search-link-from-name "l_hand"))
               (cons :joint-list (append (send robot :torso :joint-list)
                                         (send robot :rarm :joint-list)))
               )

         )
   )
  )
