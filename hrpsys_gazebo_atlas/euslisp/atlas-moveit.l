(require :atlas-interface
         "package://hrpsys_gazebo_atlas/euslisp/atlas-interface.l")
(require :pr2eus-moveit "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")

(defclass atlas-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod atlas-moveit-environment
  (:init
   (&key ((:robot rb)))
   (unless rb
     (cond
      ((boundp '*atlas*)
       (let ((tmp *atlas*))
         (setq rb (atlas))
         (setq *atlas* tmp)))
      (t (setq rb (atlas)))))
   (send-super :init :robot rb
               :frame-id "pelvis"
               :multi-dof-joint-name "virtual_joint"
               :multi-dof-frame-id   "/map") ;; /world should be published
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
;; gazebo
(atlas-init-ex)
(send *ri* :set-moveit-environment
      (instance atlas-moveit-environment :init)) ;; call (atlas) inside it
;;
;; just planning
(send *ri* :angle-vector-make-trajectory (send *atlas* :angle-vector) :move-arm :larm :use-torso nil)
;;
;; move
(send *ri* :angle-vector-motion-plan (send *atlas* :angle-vector) :move-arm :larm :use-torso nil)
|#

#|
;; simulation
(setq *co* (instance collision-object-publisher :init))
(setq *me* (instance atlas-moveit-environment :init)) ;; call (atlas) inside it

(if (not (boundp '*ri*)) (setq *ri* (instance robot-interface :init :robot (atlas) :namespace "ri")))
(setq *atlas* (atlas-with-hand)))
(send *atlas* :fix-leg-to-coords (make-coords))
(objects (list *atlas*))

;; motion plan
(send *ri* :angle-vector (send *atlas* :reset-pose)) ;; set robot joint-angle (start state)
(send *ri* :angle-vector (send *atlas* :angle-vector)) ;; set robot joint-angle (start state)

;; add collision object
(send *co* :wipe-all)
(setq cb0 (make-cube 10 2400 2000))
(send cb0 :translate #f(480 0 1000))
(setq cb1 (make-cube 2400 10 2000))
(send cb1 :translate #f(0 -580 1000))
(setq cb2 (make-cube 2400 10 2000))
(send cb2 :translate #f(0 580 1000))
(objects (list *atlas* cb0 cb1 cb2))

(publish-collision-object cb0 *atlas* "pelvis")
(publish-collision-object cb1 *atlas* "pelvis")
(publish-collision-object cb2 *atlas* "pelvis")

(send *co* :wipe-all)
(setq cb3 (make-cube 40 1000 2000))
(send cb3 :translate (float-vector 450 (+ 500 (/ 914.4 2)) 1000))
(setq cb4 (make-cube 40 1000 2000))
(send cb4 :translate (float-vector 450 (- -500 (/ 914.4 2)) 1000))
(objects (list *atlas* cb3 cb4))
(publish-collision-object cb3 *atlas* "pelvis")
(publish-collision-object cb4 *atlas* "pelvis")

(send *atlas* :pre-manip-pose
;;(send *atlas* :reset-manip-pose)
(send *me* :sync-robot-model *atlas*) ;; set target-pose to moveit
(setq ret (send *me* :motion-plan :larm :planning-attempts 10 :planning-time 200.0))
(setq avs (apply-joint_trajectory (send ret :trajectory :joint_trajectory) *atlas*))

(send *me* :motion-plan :rarm-torso)
;;
(load "package://hrpsys_gazebo_atlas/models/drc-vehicle-simple.l")

(send *atlas* :fix-leg-to-coords (make-coords))
(send *drc-car* :translate (float-vector 0 1200 0))
(objects (list *atlas* *drc-car*))

(publish-collision-object *drc-car* *atlas* "pelvis")

;; collision aware ik
(send *me* :sync-robot-model *atlas*) ;; set initial-pose to moveit
(send *me* :get-ik-for-pose
      (send *atlas* :larm :end-coords :copy-worldcoords)
      :larm :end-coords (list :larm :end-coords))
(send *me* :sync-robot-model *atlas* t) ;; set ik result to *atlas*
|#