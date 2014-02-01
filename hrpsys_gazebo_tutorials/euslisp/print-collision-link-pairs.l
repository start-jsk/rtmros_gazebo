(load "package://hrpsys_gazebo_tutorials/euslisp/hrp2jsknt-interface-ex.l")


(init :real nil)

(let* (link-pair-list ;; list of link pair
       ignore-link-pair-string ;; list of strings which include ignore link pair name
       (link-name-list (send-all (send *robot* :links) :name)))

  ;; make link pair
  (dolist (l1 link-name-list)
    (dolist (l2 (cdr (member l1 link-name-list :test 'string=)))
      (push (list l1 l2) link-pair-list)
      ))
  ;;(pprint link-pair-list);;
  (warn "number of all link pairs is ~a~%" (length link-pair-list))

  ;; make ignore link pair from moveit srdf file
  (let* ((res-stream
	  (piped-fork "grep disable_collisions `rospack find hrp2jsknt_moveit_config`/config/HRP2JSKNT.srdf"))
	 res-string
	 res-tmp)
    (while (setq res-tmp (read-line res-stream nil))
      (push res-tmp res-string))
    (warn "file lines ~a~%" (length res-string))  
    (setq res-string
	  (remove-if-not #'(lambda (x) (substringp "disable_collisions" x)) res-string))
    (setq ignore-link-pair-string res-string)
    )
  ;;(pprint ignore-link-pair-string);;
  (warn "number of ignore link pairs is ~a~%" (length ignore-link-pair-string))  

  ;; make collision check link pair
  (setq link-pair-list
	(remove-if
	 #'(lambda (lp)
	     (position t
		       (mapcar #'(lambda (res-l) (and (substringp (format nil "~A" (elt lp 0)) res-l)
						      (substringp (format nil "~A" (elt lp 1)) res-l))) ignore-link-pair-string)))
	 link-pair-list))
  ;;(pprint link-pair-list);;
  (warn "number of collision check link pairs is ~a~%" (length link-pair-list))  
  
  ;; remove pair which include specified link because collision check pair is too numerous
  (setq link-pair-list
	(remove-if 
	 #'(lambda (lp)
	     (position t
		       (mapcar
			#'(lambda (limb-name)
			    (or (string= limb-name (elt lp 0)) (string= limb-name (elt lp 1))))
			(list "CHEST_LINK0" "HEAD_LINK0" "HEAD_LINK1" "RARM_LINK0" "RARM_LINK1" "RARM_LINK2" "RARM_LINK4" "RARM_LINK5" "RARM_LINK7" "RLEG_LINK0" "RLEG_LINK1" "RLEG_LINK4" "RLEG_LINK5" "RLEG_LINK6" "LARM_LINK0" "LARM_LINK1" "LARM_LINK2" "LARM_LINK4" "LARM_LINK5" "LARM_LINK7" "LLEG_LINK0" "LLEG_LINK1" "LLEG_LINK4" "LLEG_LINK5" "LLEG_LINK6"))))
	 link-pair-list))
  (warn "number of collision check link pairs after removing heuristic pairs is ~a~%" (length link-pair-list))  

  ;; remove specified pair  because collision check pair is too numerous
  (setq link-pair-list
	(remove-if 
	 #'(lambda (lp)
	     (position t
		       (mapcar
			#'(lambda (limb-name-list)
			    (or (and (string= (elt limb-name-list 0) (elt lp 0)) (string= (elt limb-name-list 1) (elt lp 1)))
				(and (string= (elt limb-name-list 1) (elt lp 0)) (string= (elt limb-name-list 0) (elt lp 1)))))
			(list (list "BODY" "RLEG_LINK2") (list "BODY" "LLEG_LINK2")))))
	 link-pair-list))
  (warn "number of collision check link pairs after removing heuristic pairs is ~a~%" (length link-pair-list))

  ;; remove pair which belongs to the same limb because collision check pair is too numerous
  (setq link-pair-list
	(remove-if 
	 #'(lambda (lp)
	     (position t
		       (mapcar
			#'(lambda (limb-name)
			    (cond ((string= limb-name "CHEST")
				   (or (and (substringp limb-name (elt lp 0)) (substringp limb-name (elt lp 1)))
				       (and (substringp limb-name (elt lp 0)) (substringp "BODY" (elt lp 1)))
				       (and (substringp "BODY" (elt lp 0)) (substringp limb-name (elt lp 1)))))
				  (t
				   (and (substringp limb-name (elt lp 0)) (substringp limb-name (elt lp 1))))))
			(list "LARM" "RARM" "LLEG" "RLEG" "HEAD" "CHEST"))))
	 link-pair-list))
  (warn "number of collision check link pairs after removing heuristic pairs is ~a~%" (length link-pair-list))  

  ;; remove pair which belongs to the other side leg and arm because collision check pair is too numerous
  (setq link-pair-list
	(remove-if 
	 #'(lambda (lp)
	     (position t
		       (mapcar
			#'(lambda (limb-name-list)
			    (or (and (substringp (elt limb-name-list 0) (elt lp 0)) (substringp (elt limb-name-list 1) (elt lp 1)))
				(and (substringp (elt limb-name-list 1) (elt lp 0)) (substringp (elt limb-name-list 0) (elt lp 1)))
				(and (substringp (elt limb-name-list 0) (elt lp 1)) (substringp (elt limb-name-list 1) (elt lp 0)))
				(and (substringp (elt limb-name-list 1) (elt lp 1)) (substringp (elt limb-name-list 0) (elt lp 0)))))
			(list (list "LARM" "RLEG") (list "RARM" "LLEG")))))
	 link-pair-list))
  (warn "number of collision check link pairs after removing heuristic pairs is ~a~%" (length link-pair-list))  

  ;; convert to joint name from link name
  (setq link-pair-list
	(mapcar
	 #'(lambda (lp)
	     (mapcar
	      #'(lambda (l)
		  (cond ((string= l "BODY") "WAIST")
			(t (send (send (send *robot* :link l) :joint) :name))))
	      lp))
	 link-pair-list))
  ;;(pprint link-pair-list);;

  ;; output
  (format t "--conf-file-option \"collision_pair:")
  (dolist (lp link-pair-list)
    (format t " ~a:~a" (elt lp 0) (elt lp 1)))
  (format t "\"~%")
  )

