;; Auto-generated. Do not edit!


(when (boundp 'odin_ros_driver::GetAe)
  (if (not (find-package "ODIN_ROS_DRIVER"))
    (make-package "ODIN_ROS_DRIVER"))
  (shadow 'GetAe (find-package "ODIN_ROS_DRIVER")))
(unless (find-package "ODIN_ROS_DRIVER::GETAE")
  (make-package "ODIN_ROS_DRIVER::GETAE"))
(unless (find-package "ODIN_ROS_DRIVER::GETAEREQUEST")
  (make-package "ODIN_ROS_DRIVER::GETAEREQUEST"))
(unless (find-package "ODIN_ROS_DRIVER::GETAERESPONSE")
  (make-package "ODIN_ROS_DRIVER::GETAERESPONSE"))

(in-package "ROS")





(defclass odin_ros_driver::GetAeRequest
  :super ros::object
  :slots ())

(defmethod odin_ros_driver::GetAeRequest
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

(defclass odin_ros_driver::GetAeResponse
  :super ros::object
  :slots (_success _rc _exposure_time _gain _iso _brightness _is_converged _env_lv _fps ))

(defmethod odin_ros_driver::GetAeResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:rc __rc) 0)
    ((:exposure_time __exposure_time) 0.0)
    ((:gain __gain) 0.0)
    ((:iso __iso) 0)
    ((:brightness __brightness) 0.0)
    ((:is_converged __is_converged) 0)
    ((:env_lv __env_lv) 0.0)
    ((:fps __fps) 0.0)
    )
   (send-super :init)
   (setq _success __success)
   (setq _rc (round __rc))
   (setq _exposure_time (float __exposure_time))
   (setq _gain (float __gain))
   (setq _iso (round __iso))
   (setq _brightness (float __brightness))
   (setq _is_converged (round __is_converged))
   (setq _env_lv (float __env_lv))
   (setq _fps (float __fps))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:rc
   (&optional __rc)
   (if __rc (setq _rc __rc)) _rc)
  (:exposure_time
   (&optional __exposure_time)
   (if __exposure_time (setq _exposure_time __exposure_time)) _exposure_time)
  (:gain
   (&optional __gain)
   (if __gain (setq _gain __gain)) _gain)
  (:iso
   (&optional __iso)
   (if __iso (setq _iso __iso)) _iso)
  (:brightness
   (&optional __brightness)
   (if __brightness (setq _brightness __brightness)) _brightness)
  (:is_converged
   (&optional __is_converged)
   (if __is_converged (setq _is_converged __is_converged)) _is_converged)
  (:env_lv
   (&optional __env_lv)
   (if __env_lv (setq _env_lv __env_lv)) _env_lv)
  (:fps
   (&optional __fps)
   (if __fps (setq _fps __fps)) _fps)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; int32 _rc
    4
    ;; float32 _exposure_time
    4
    ;; float32 _gain
    4
    ;; int32 _iso
    4
    ;; float32 _brightness
    4
    ;; uint8 _is_converged
    1
    ;; float32 _env_lv
    4
    ;; float32 _fps
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
     ;; float32 _exposure_time
       (sys::poke _exposure_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gain
       (sys::poke _gain (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _iso
       (write-long _iso s)
     ;; float32 _brightness
       (sys::poke _brightness (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _is_converged
       (write-byte _is_converged s)
     ;; float32 _env_lv
       (sys::poke _env_lv (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _fps
       (sys::poke _fps (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; int32 _rc
     (setq _rc (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _exposure_time
     (setq _exposure_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gain
     (setq _gain (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _iso
     (setq _iso (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _brightness
     (setq _brightness (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _is_converged
     (setq _is_converged (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; float32 _env_lv
     (setq _env_lv (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _fps
     (setq _fps (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass odin_ros_driver::GetAe
  :super ros::object
  :slots ())

(setf (get odin_ros_driver::GetAe :md5sum-) "542c4f6644aaf51ceda9dea371da93c3")
(setf (get odin_ros_driver::GetAe :datatype-) "odin_ros_driver/GetAe")
(setf (get odin_ros_driver::GetAe :request) odin_ros_driver::GetAeRequest)
(setf (get odin_ros_driver::GetAe :response) odin_ros_driver::GetAeResponse)

(defmethod odin_ros_driver::GetAeRequest
  (:response () (instance odin_ros_driver::GetAeResponse :init)))

(setf (get odin_ros_driver::GetAeRequest :md5sum-) "542c4f6644aaf51ceda9dea371da93c3")
(setf (get odin_ros_driver::GetAeRequest :datatype-) "odin_ros_driver/GetAeRequest")
(setf (get odin_ros_driver::GetAeRequest :definition-)
      "# Query current AE (auto exposure) state.
# Request: empty.
# Response: success + raw return code + decoded fields.
---
bool success            # true if rc == 0
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error
float32 exposure_time   # seconds (manual range 0.0001 .. 0.033)
float32 gain            # analog gain
int32 iso               # equivalent ISO
float32 brightness      # average frame brightness
uint8 is_converged      # 1=converged, 0=not converged
float32 env_lv          # ambient luminance level
float32 fps             # current frame rate

")

(setf (get odin_ros_driver::GetAeResponse :md5sum-) "542c4f6644aaf51ceda9dea371da93c3")
(setf (get odin_ros_driver::GetAeResponse :datatype-) "odin_ros_driver/GetAeResponse")
(setf (get odin_ros_driver::GetAeResponse :definition-)
      "# Query current AE (auto exposure) state.
# Request: empty.
# Response: success + raw return code + decoded fields.
---
bool success            # true if rc == 0
int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error
float32 exposure_time   # seconds (manual range 0.0001 .. 0.033)
float32 gain            # analog gain
int32 iso               # equivalent ISO
float32 brightness      # average frame brightness
uint8 is_converged      # 1=converged, 0=not converged
float32 env_lv          # ambient luminance level
float32 fps             # current frame rate

")



(provide :odin_ros_driver/GetAe "542c4f6644aaf51ceda9dea371da93c3")


