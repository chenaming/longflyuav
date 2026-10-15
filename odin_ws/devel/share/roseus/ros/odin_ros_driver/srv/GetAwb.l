;; Auto-generated. Do not edit!


(when (boundp 'odin_ros_driver::GetAwb)
  (if (not (find-package "ODIN_ROS_DRIVER"))
    (make-package "ODIN_ROS_DRIVER"))
  (shadow 'GetAwb (find-package "ODIN_ROS_DRIVER")))
(unless (find-package "ODIN_ROS_DRIVER::GETAWB")
  (make-package "ODIN_ROS_DRIVER::GETAWB"))
(unless (find-package "ODIN_ROS_DRIVER::GETAWBREQUEST")
  (make-package "ODIN_ROS_DRIVER::GETAWBREQUEST"))
(unless (find-package "ODIN_ROS_DRIVER::GETAWBRESPONSE")
  (make-package "ODIN_ROS_DRIVER::GETAWBRESPONSE"))

(in-package "ROS")





(defclass odin_ros_driver::GetAwbRequest
  :super ros::object
  :slots ())

(defmethod odin_ros_driver::GetAwbRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass odin_ros_driver::GetAwbResponse
  :super ros::object
  :slots (_success _rc _rgain _grgain _gbgain _bgain _cct _ccri _is_converged ))

(defmethod odin_ros_driver::GetAwbResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:rc __rc) 0)
    ((:rgain __rgain) 0.0)
    ((:grgain __grgain) 0.0)
    ((:gbgain __gbgain) 0.0)
    ((:bgain __bgain) 0.0)
    ((:cct __cct) 0.0)
    ((:ccri __ccri) 0.0)
    ((:is_converged __is_converged) 0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _rc (round __rc))
   (setq _rgain (float __rgain))
   (setq _grgain (float __grgain))
   (setq _gbgain (float __gbgain))
   (setq _bgain (float __bgain))
   (setq _cct (float __cct))
   (setq _ccri (float __ccri))
   (setq _is_converged (round __is_converged))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:rc
   (&optional __rc)
   (if __rc (setq _rc __rc)) _rc)
  (:rgain
   (&optional __rgain)
   (if __rgain (setq _rgain __rgain)) _rgain)
  (:grgain
   (&optional __grgain)
   (if __grgain (setq _grgain __grgain)) _grgain)
  (:gbgain
   (&optional __gbgain)
   (if __gbgain (setq _gbgain __gbgain)) _gbgain)
  (:bgain
   (&optional __bgain)
   (if __bgain (setq _bgain __bgain)) _bgain)
  (:cct
   (&optional __cct)
   (if __cct (setq _cct __cct)) _cct)
  (:ccri
   (&optional __ccri)
   (if __ccri (setq _ccri __ccri)) _ccri)
  (:is_converged
   (&optional __is_converged)
   (if __is_converged (setq _is_converged __is_converged)) _is_converged)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _rc
    4
    ;; float32 _rgain
    4
    ;; float32 _grgain
    4
    ;; float32 _gbgain
    4
    ;; float32 _bgain
    4
    ;; float32 _cct
    4
    ;; float32 _ccri
    4
    ;; uint8 _is_converged
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; int32 _rc
       (write-long _rc s)
     ;; float32 _rgain
       (sys::poke _rgain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _grgain
       (sys::poke _grgain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gbgain
       (sys::poke _gbgain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _bgain
       (sys::poke _bgain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _cct
       (sys::poke _cct (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ccri
       (sys::poke _ccri (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _is_converged
       (write-byte _is_converged s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _rc
     (setq _rc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _rgain
     (setq _rgain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _grgain
     (setq _grgain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gbgain
     (setq _gbgain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _bgain
     (setq _bgain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _cct
     (setq _cct (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ccri
     (setq _ccri (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _is_converged
     (setq _is_converged (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass odin_ros_driver::GetAwb
  :super ros::object
  :slots ())

(setf (get odin_ros_driver::GetAwb :md5sum-) "508f4cd8169f95ac5caf4673a24ce5b2")
(setf (get odin_ros_driver::GetAwb :datatype-) "odin_ros_driver/GetAwb")
(setf (get odin_ros_driver::GetAwb :request) odin_ros_driver::GetAwbRequest)
(setf (get odin_ros_driver::GetAwb :response) odin_ros_driver::GetAwbResponse)

(defmethod odin_ros_driver::GetAwbRequest
  (:response () (instance odin_ros_driver::GetAwbResponse :init)))

(setf (get odin_ros_driver::GetAwbRequest :md5sum-) "508f4cd8169f95ac5caf4673a24ce5b2")
(setf (get odin_ros_driver::GetAwbRequest :datatype-) "odin_ros_driver/GetAwbRequest")
(setf (get odin_ros_driver::GetAwbRequest :definition-)
      "# Query current AWB (auto white balance) state.
# Request: empty.
# Response: success + raw return code + decoded fields.
---
bool success            # true if rc == 0
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error
float32 rgain
float32 grgain
float32 gbgain
float32 bgain
float32 cct             # color temperature in Kelvin
float32 ccri            # color temperature deviation
uint8 is_converged      # 1=converged, 0=not converged

")

(setf (get odin_ros_driver::GetAwbResponse :md5sum-) "508f4cd8169f95ac5caf4673a24ce5b2")
(setf (get odin_ros_driver::GetAwbResponse :datatype-) "odin_ros_driver/GetAwbResponse")
(setf (get odin_ros_driver::GetAwbResponse :definition-)
      "# Query current AWB (auto white balance) state.
# Request: empty.
# Response: success + raw return code + decoded fields.
---
bool success            # true if rc == 0
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error
float32 rgain
float32 grgain
float32 gbgain
float32 bgain
float32 cct             # color temperature in Kelvin
float32 ccri            # color temperature deviation
uint8 is_converged      # 1=converged, 0=not converged

")



(provide :odin_ros_driver/GetAwb "508f4cd8169f95ac5caf4673a24ce5b2")


