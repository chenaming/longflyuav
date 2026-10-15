;; Auto-generated. Do not edit!


(when (boundp 'odin_ros_driver::SetAwb)
  (if (not (find-package "ODIN_ROS_DRIVER"))
    (make-package "ODIN_ROS_DRIVER"))
  (shadow 'SetAwb (find-package "ODIN_ROS_DRIVER")))
(unless (find-package "ODIN_ROS_DRIVER::SETAWB")
  (make-package "ODIN_ROS_DRIVER::SETAWB"))
(unless (find-package "ODIN_ROS_DRIVER::SETAWBREQUEST")
  (make-package "ODIN_ROS_DRIVER::SETAWBREQUEST"))
(unless (find-package "ODIN_ROS_DRIVER::SETAWBRESPONSE")
  (make-package "ODIN_ROS_DRIVER::SETAWBRESPONSE"))

(in-package "ROS")





(defclass odin_ros_driver::SetAwbRequest
  :super ros::object
  :slots (_mode _rgain _bgain ))

(defmethod odin_ros_driver::SetAwbRequest
  (:init
   (&key
    ((:mode __mode) 0)
    ((:rgain __rgain) 0.0)
    ((:bgain __bgain) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _rgain (float __rgain))
   (setq _bgain (float __bgain))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:rgain
   (&optional __rgain)
   (if __rgain (setq _rgain __rgain)) _rgain)
  (:bgain
   (&optional __bgain)
   (if __bgain (setq _bgain __bgain)) _bgain)
  (:serialization-length
   ()
   (+
    ;; uint8 _mode
    1
    ;; float32 _rgain
    4
    ;; float32 _bgain
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _mode
       (write-byte _mode s)
     ;; float32 _rgain
       (sys::poke _rgain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bgain
       (sys::poke _bgain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _rgain
     (setq _rgain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bgain
     (setq _bgain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass odin_ros_driver::SetAwbResponse
  :super ros::object
  :slots (_success _rc ))

(defmethod odin_ros_driver::SetAwbResponse
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

(defclass odin_ros_driver::SetAwb
  :super ros::object
  :slots ())

(setf (get odin_ros_driver::SetAwb :md5sum-) "b1976cc70ace03d84cebe2d1a9bc817e")
(setf (get odin_ros_driver::SetAwb :datatype-) "odin_ros_driver/SetAwb")
(setf (get odin_ros_driver::SetAwb :request) odin_ros_driver::SetAwbRequest)
(setf (get odin_ros_driver::SetAwb :response) odin_ros_driver::SetAwbResponse)

(defmethod odin_ros_driver::SetAwbRequest
  (:response () (instance odin_ros_driver::SetAwbResponse :init)))

(setf (get odin_ros_driver::SetAwbRequest :md5sum-) "b1976cc70ace03d84cebe2d1a9bc817e")
(setf (get odin_ros_driver::SetAwbRequest :datatype-) "odin_ros_driver/SetAwbRequest")
(setf (get odin_ros_driver::SetAwbRequest :definition-)
      "# Set AWB (auto white balance) mode and (in manual mode) R/B gains.
#
# mode == 0 (AUTO)   : sends opcode 0x31 only; rgain and bgain ignored.
# mode == 1 (MANUAL) : sends opcode 0x32 then 0x33 (rgain, bgain).
#                     Gr/Gb are fixed to 1.0 by the device.
#
# Valid manual ranges:
#   rgain : 0.1 .. 4.0
#   bgain : 0.1 .. 4.0
uint8 mode
float32 rgain
float32 bgain
---
bool success
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error

")

(setf (get odin_ros_driver::SetAwbResponse :md5sum-) "b1976cc70ace03d84cebe2d1a9bc817e")
(setf (get odin_ros_driver::SetAwbResponse :datatype-) "odin_ros_driver/SetAwbResponse")
(setf (get odin_ros_driver::SetAwbResponse :definition-)
      "# Set AWB (auto white balance) mode and (in manual mode) R/B gains.
#
# mode == 0 (AUTO)   : sends opcode 0x31 only; rgain and bgain ignored.
# mode == 1 (MANUAL) : sends opcode 0x32 then 0x33 (rgain, bgain).
#                     Gr/Gb are fixed to 1.0 by the device.
#
# Valid manual ranges:
#   rgain : 0.1 .. 4.0
#   bgain : 0.1 .. 4.0
uint8 mode
float32 rgain
float32 bgain
---
bool success
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error

")



(provide :odin_ros_driver/SetAwb "b1976cc70ace03d84cebe2d1a9bc817e")


