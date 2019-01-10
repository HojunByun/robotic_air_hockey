;; Auto-generated. Do not edit!


(when (boundp 'airhockey_main::ArmAngles)
  (if (not (find-package "AIRHOCKEY_MAIN"))
    (make-package "AIRHOCKEY_MAIN"))
  (shadow 'ArmAngles (find-package "AIRHOCKEY_MAIN")))
(unless (find-package "AIRHOCKEY_MAIN::ARMANGLES")
  (make-package "AIRHOCKEY_MAIN::ARMANGLES"))

(in-package "ROS")
;;//! \htmlinclude ArmAngles.msg.html


(defclass airhockey_main::ArmAngles
  :super ros::object
  :slots (_arm0_joint0 _arm0_joint1 _arm1_joint0 _arm1_joint1 _success ))

(defmethod airhockey_main::ArmAngles
  (:init
   (&key
    ((:arm0_joint0 __arm0_joint0) 0.0)
    ((:arm0_joint1 __arm0_joint1) 0.0)
    ((:arm1_joint0 __arm1_joint0) 0.0)
    ((:arm1_joint1 __arm1_joint1) 0.0)
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _arm0_joint0 (float __arm0_joint0))
   (setq _arm0_joint1 (float __arm0_joint1))
   (setq _arm1_joint0 (float __arm1_joint0))
   (setq _arm1_joint1 (float __arm1_joint1))
   (setq _success __success)
   self)
  (:arm0_joint0
   (&optional __arm0_joint0)
   (if __arm0_joint0 (setq _arm0_joint0 __arm0_joint0)) _arm0_joint0)
  (:arm0_joint1
   (&optional __arm0_joint1)
   (if __arm0_joint1 (setq _arm0_joint1 __arm0_joint1)) _arm0_joint1)
  (:arm1_joint0
   (&optional __arm1_joint0)
   (if __arm1_joint0 (setq _arm1_joint0 __arm1_joint0)) _arm1_joint0)
  (:arm1_joint1
   (&optional __arm1_joint1)
   (if __arm1_joint1 (setq _arm1_joint1 __arm1_joint1)) _arm1_joint1)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; float32 _arm0_joint0
    4
    ;; float32 _arm0_joint1
    4
    ;; float32 _arm1_joint0
    4
    ;; float32 _arm1_joint1
    4
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _arm0_joint0
       (sys::poke _arm0_joint0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _arm0_joint1
       (sys::poke _arm0_joint1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _arm1_joint0
       (sys::poke _arm1_joint0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _arm1_joint1
       (sys::poke _arm1_joint1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _arm0_joint0
     (setq _arm0_joint0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _arm0_joint1
     (setq _arm0_joint1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _arm1_joint0
     (setq _arm1_joint0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _arm1_joint1
     (setq _arm1_joint1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get airhockey_main::ArmAngles :md5sum-) "03c786ea5bb5b9ff9771f16c1a88ee81")
(setf (get airhockey_main::ArmAngles :datatype-) "airhockey_main/ArmAngles")
(setf (get airhockey_main::ArmAngles :definition-)
      "float32 arm0_joint0
float32 arm0_joint1
float32 arm1_joint0
float32 arm1_joint1
bool success

")



(provide :airhockey_main/ArmAngles "03c786ea5bb5b9ff9771f16c1a88ee81")


