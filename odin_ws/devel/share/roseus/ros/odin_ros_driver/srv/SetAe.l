;; Auto-generated. Do not edit!


(when (boundp 'odin_ros_driver::SetAe)
  (if (not (find-package "ODIN_ROS_DRIVER"))
    (make-package "ODIN_ROS_DRIVER"))
  (shadow 'SetAe (find-package "ODIN_ROS_DRIVER")))
(unless (find-package "ODIN_ROS_DRIVER::SETAE")
  (make-package "ODIN_ROS_DRIVER::SETAE"))
(unless (find-package "ODIN_ROS_DRIVER::SETAEREQUEST")
  (make-package "ODIN_ROS_DRIVER::SETAEREQUEST"))
(unless (find-package "ODIN_ROS_DRIVER::SETAERESPONSE")
  (make-package "ODIN_ROS_DRIVER::SETAERESPONSE"))

(in-package "ROS")





(defclass odin_ros_driver::SetAeRequest
  :super ros::object
  :slots (_mode _exposure_time _gain ))

(defmethod odin_ros_driver::SetAeRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:exposure_time __exposure_time) 0.0)
    ((:gain __gain) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _exposure_time (float __exposure_time))
   (setq _gain (float __gain))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:exposure_time
   (&optional __exposure_time)
   (if __exposure_time (setq _exposure_time __exposure_time)) _exposure_time)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; float32 _exposure_time
    4
    ;; float32 _gain
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; float32 _exposure_time
       (sys::poke _exposure_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gain
       (sys::poke _gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _exposure_time
     (setq _exposure_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gain
     (setq _gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass odin_ros_driver::SetAeResponse
  :super ros::object
  :slots (_success _rc ))

(defmethod odin_ros_driver::SetAeResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:rc __rc) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _rc (round __rc))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:rc
   (&optional __rc)
   (if __rc (setq _rc __rc)) _rc)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _rc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _rc
       (write-long _rc s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _rc
     (setq _rc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass odin_ros_driver::SetAe
  :super ros::object
  :slots ())

(setf (get odin_ros_driver::SetAe :md5sum-) "a79a03de733d9c62ec6c16126efd1953")
(setf (get odin_ros_driver::SetAe :datatype-) "odin_ros_driver/SetAe")
(setf (get odin_ros_driver::SetAe :request) odin_ros_driver::SetAeRequest)
(setf (get odin_ros_driver::SetAe :response) odin_ros_driver::SetAeResponse)

(defmethod odin_ros_driver::SetAeRequest
  (:response () (instance odin_ros_driver::SetAeResponse :init)))

(setf (get odin_ros_driver::SetAeRequest :md5sum-) "a79a03de733d9c62ec6c16126efd1953")
(setf (get odin_ros_driver::SetAeRequest :datatype-) "odin_ros_driver/SetAeRequest")
(setf (get odin_ros_driver::SetAeRequest :definition-)
      "# Set AE (auto exposure) mode and (in manual mode) exposure / gain.
#
# mode == 0 (AUTO)   : sends opcode 0x02 only; exposure_time and gain ignored.
# mode == 1 (MANUAL) : sends opcode 0x03 then 0x06 (exposure_time, gain).
#
# Valid manual ranges (device-enforced; out-of-range returns rc=403):
#   exposure_time : 0.0001 s .. 0.033 s
#   gain          : 1.0     .. 64.0
uint8 mode
float32 exposure_time
float32 gain
---
bool success
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error

")

(setf (get odin_ros_driver::SetAeResponse :md5sum-) "a79a03de733d9c62ec6c16126efd1953")
(setf (get odin_ros_driver::SetAeResponse :datatype-) "odin_ros_driver/SetAeResponse")
(setf (get odin_ros_driver::SetAeResponse :definition-)
      "# Set AE (auto exposure) mode and (in manual mode) exposure / gain.
#
# mode == 0 (AUTO)   : sends opcode 0x02 only; exposure_time and gain ignored.
# mode == 1 (MANUAL) : sends opcode 0x03 then 0x06 (exposure_time, gain).
#
# Valid manual ranges (device-enforced; out-of-range returns rc=403):
#   exposure_time : 0.0001 s .. 0.033 s
#   gain          : 1.0     .. 64.0
uint8 mode
float32 exposure_time
float32 gain
---
bool success
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error

")



(provide :odin_ros_driver/SetAe "a79a03de733d9c62ec6c16126efd1953")


