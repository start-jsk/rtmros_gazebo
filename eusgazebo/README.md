### Usage
```lisp

  ;; initialize ;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;;; init eusgazebo
  (setq *eusgazebo-server* (instance eusgazebo :init))

  ;;;; generate eus model
  (setq *obj* (generate-eus-model-function))

  ;;;; add eus mdoel to eusgazebo
  (send *eusgazebo-server* :add-model *obj*)


  ;; simulate ;;;;;;;;;;;;;;;;;;;;;;;;;;
  ;;;; set simulation setting
  (send *obj* :newcoords (make-coords :pos #f(0 0 1000)))
  (send *eusgazebo-server* :pause-sim)
  (send *eusgazebo-server* :eus2gzb)

  ;;;; start simulation
  (send *eusgazebo-server* :unpause-sim)
  (send *eusgazebo-server* :gzb2eus-loop)

```

### Sample
```bash
roscore &
roscd eusgazebo/samples
roseus fall-arrow-object-simulation.l "(fall-arrow-object-simulation)"
roseus play-pinball-simulation.l "(progn (init-pinball-simulation) (play-pinball-simulation))"
roseus play-domino-simulation.l "(progn (init-domino-simulation) (play-domino-simulation))"
```

### ROS Test
```bash
rostest eusgazebo test-fall-arrow-object-simulation.test
rostest eusgazebo test-play-domino-simulation.test
```

