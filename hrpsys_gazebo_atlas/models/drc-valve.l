;;I got this valve from here!
;;https://www.monotaro.com/monotaroMain.py?func=monotaro.product.item.showDetailServlet.ShowDetailServlet&monotaroNo=37699541&displayId=3
(defclass valve
  :super cascaded-link
  :slots (handles valve-joint valve-ring-r)
  )

(defmethod valve
  (:init
    (&optional (r 135) &rest args)
    (send-super* :init args)
    (setq handles nil)
    (setq valve-ring-r r)
    ;; 1. make links links and assoc all links
    (let ((rl (send self :make-root-link))
          (vl (send self :make-valve-link)))
      (send vl :translate #f(0 0 353) :world)  ;;vavle's hight (H)
      ;; 2. assoc links
      ;;    Root link should be associated with "self".
      (send self :assoc rl)
      (send rl :assoc vl)
      ;; 3. make all joints
      ;;    Before making joints, you should :assoc all links.
      (setq valve-joint (instance rotational-joint :init
                                  :parent-link rl :child-link vl
                                  :name :crank-joint :axis :z
                                  :min *-inf* :max *inf*))
      ;; 4. define slots for robot class
      ;;    links and joint-list for cascaded-link.
      (setq links (list rl vl))
      (setq joint-list (list valve-joint))
      ;; 5. call :init-ending after defining links and joint-list and return "self"
      (send self :init-ending)
      self))
  ;; Methods to define robot links
  (:make-root-link
    ()
    (let (base-cylinder
	  (r 90)
	  (L 240)
	  (H 353)
	  (thick 30)
	  tmp
	  cylinder1
	  cylinder2
	  cylinder3
	  cylinder4
	  upper-body
	  sphere
	  rl
	  )
      ;;base cylinder
      (setq base-cylinder (make-cylinder r L))
      (send base-cylinder :rotate (deg2rad 90) :x)
      (send base-cylinder :locate (float-vector 0 (* 0.5 L) 0) :world)
      (setq tmp (make-cube (* L 2) (- L (* thick 2))  (* L 2)))
      (setq base-cylinder (body- base-cylinder tmp))
      (setq sphere (make-gdome (make-icosahedron r)))
      (setq cylinder1 (make-cylinder (* r 0.8) (- L (* 2 thick))))
      (send cylinder1 :rotate (deg2rad 90) :x)
      (send cylinder1 :locate (float-vector 0 (* 0.5 (- L (* 2 thick))) 0) :world)
      ;;
      (setq tmp (make-cylinder (* r 0.5) (* L 2)))
      (send tmp :rotate (deg2rad 90) :x)
      (send tmp :locate (float-vector 0 0 (* -1 L)))
      (setq base-cylinder (body- base-cylinder tmp))
      (setq cylinder1 (body- cylinder1 tmp))
      ;;(setq sphere (body- sphere tmp))
      ;;
      (setq cylinder2 (make-cylinder r thick))
      (send cylinder2 :locate (float-vector 0 0 r))
      (setq cylinder3 (make-cylinder (* r 0.8) (* thick 2)))
      (send cylinder3 :locate (float-vector 0 0 (- r (* 2 thick))))
      ;;
      (setq cylinder4 (make-cylinder 20 H))
      ;;(send cylinder4 :locate (float-vector 0 0 (* 0.5 H)))
      ;;upper body
      (setq upper-body 
	    (make-solid-of-revolution
	     (mapcar #'(lambda (x) (scale (* 0.7 (/ (- H r) 3.5)) x)) (list #f(0 0 3.5) #f(0.6 0 3) #f(1.0 0 2) #f(1.4 0 1) #f(1.0 0 0)))))
      (send upper-body :locate (float-vector 0 0 (+ r thick)))
      ;;
      (send base-cylinder :assoc sphere)
      (send base-cylinder :assoc cylinder1)
      (send base-cylinder :assoc cylinder2)
      (send base-cylinder :assoc cylinder3)
      (send base-cylinder :assoc cylinder4)
      (send base-cylinder :assoc upper-body)
      (setq rl (instance bodyset-link :init (make-cascoords)
		:bodies (list base-cylinder sphere cylinder1 cylinder2 cylinder3 upper-body cylinder4)
		:name :crank-root-link))
      (dolist (l (send rl :bodies))
	(send l :set-color :royalblue))
      rl
      ))
  (:make-valve-link
    ()
    (let* (b0 b1 b2 vl ahandle bhandle
	    v-list
	   (r valve-ring-r)
	   (div 10)
	   (valve-r 20)
	   )
      (dotimes (i div)
	(push (float-vector (+ (* valve-r (cos (* i (/ 2pi div)))) r) 0  
			    (* valve-r (sin (* i (/ 2pi div))))) v-list))
      (setq b0 (make-torus v-list))
      (setq b1 (make-cube valve-r (* r 2) valve-r ))
      (setq b2 (make-cube (* r 2) valve-r valve-r ))
      (send b0 :assoc b1)
      (send b0 :assoc b2)
      (setq vl (instance bodyset-link :init (make-cascoords)
			:bodies (list b0 b1 b2) :name :valve-handle-link))
      ;;??
      (setq ahandle (make-cascoords :coords (send (send b0 :copy-worldcoords) :translate (float-vector 0 r 0))
				    :name :valve-handle))
      (send vl :assoc ahandle)
      (dolist (l (send vl :bodies))
	(send l :set-color :gray10))
      (push ahandle handles)
      (setq bhandle (make-cascoords :coords (send (send b0 :copy-worldcoords) :translate (float-vector 0 (- r) 0))
				    :name :valve-handle))
      (send vl :assoc bhandle)
      (dolist (l (send vl :bodies))
	(send l :set-color :gray10))
      (push bhandle handles)
      (print vl)
      vl))
  (:valve-handle0 () (elt handles 0))
  (:valve-handle1 () (elt handles 1))
  (:valve-joint (&rest args) (forward-message-to valve-joint args))
  )

(defun valve-test
  ()
  (setq va (instance valve :init))
  (objects (list va))
  )

