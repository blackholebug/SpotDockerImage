;; Auto-generated. Do not edit!


(when (boundp 'spot_fsm_control_msgs::Action)
  (if (not (find-package "SPOT_FSM_CONTROL_MSGS"))
    (make-package "SPOT_FSM_CONTROL_MSGS"))
  (shadow 'Action (find-package "SPOT_FSM_CONTROL_MSGS")))
(unless (find-package "SPOT_FSM_CONTROL_MSGS::ACTION")
  (make-package "SPOT_FSM_CONTROL_MSGS::ACTION"))

(in-package "ROS")
;;//! \htmlinclude Action.msg.html


(defclass spot_fsm_control_msgs::Action
  :super ros::object
  :slots (_action ))

(defmethod spot_fsm_control_msgs::Action
  (:init
   (&key
    ((:action __action) "")
    )
   (send-super :init)
   (setq _action (string __action))
   self)
  (:action
   (&optional __action)
   (if __action (setq _action __action)) _action)
  (:serialization-length
   ()
   (+
    ;; string _action
    4 (length _action)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _action
       (write-long (length _action) s) (princ _action s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _action
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _action (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get spot_fsm_control_msgs::Action :md5sum-) "7757aad79fa343e61bc69ed7f1b7666d")
(setf (get spot_fsm_control_msgs::Action :datatype-) "spot_fsm_control_msgs/Action")
(setf (get spot_fsm_control_msgs::Action :definition-)
      "string action
")



(provide :spot_fsm_control_msgs/Action "7757aad79fa343e61bc69ed7f1b7666d")


