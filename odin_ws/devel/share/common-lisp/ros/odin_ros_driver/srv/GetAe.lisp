; Auto-generated. Do not edit!


(cl:in-package odin_ros_driver-srv)


;//! \htmlinclude GetAe-request.msg.html

(cl:defclass <GetAe-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetAe-request (<GetAe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<GetAe-request> is deprecated: use odin_ros_driver-srv:GetAe-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAe-request>) ostream)
  "Serializes a message object of type '<GetAe-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAe-request>) istream)
  "Deserializes a message object of type '<GetAe-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAe-request>)))
  "Returns string type for a service object of type '<GetAe-request>"
  "odin_ros_driver/GetAeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAe-request)))
  "Returns string type for a service object of type 'GetAe-request"
  "odin_ros_driver/GetAeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAe-request>)))
  "Returns md5sum for a message object of type '<GetAe-request>"
  "542c4f6644aaf51ceda9dea371da93c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAe-request)))
  "Returns md5sum for a message object of type 'GetAe-request"
  "542c4f6644aaf51ceda9dea371da93c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAe-request>)))
  "Returns full string definition for message of type '<GetAe-request>"
  (cl:format cl:nil "# Query current AE (auto exposure) state.~%# Request: empty.~%# Response: success + raw return code + decoded fields.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAe-request)))
  "Returns full string definition for message of type 'GetAe-request"
  (cl:format cl:nil "# Query current AE (auto exposure) state.~%# Request: empty.~%# Response: success + raw return code + decoded fields.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAe-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAe-request
))
;//! \htmlinclude GetAe-response.msg.html

(cl:defclass <GetAe-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (rc
    :reader rc
    :initarg :rc
    :type cl:integer
    :initform 0)
   (exposure_time
    :reader exposure_time
    :initarg :exposure_time
    :type cl:float
    :initform 0.0)
   (gain
    :reader gain
    :initarg :gain
    :type cl:float
    :initform 0.0)
   (iso
    :reader iso
    :initarg :iso
    :type cl:integer
    :initform 0)
   (brightness
    :reader brightness
    :initarg :brightness
    :type cl:float
    :initform 0.0)
   (is_converged
    :reader is_converged
    :initarg :is_converged
    :type cl:fixnum
    :initform 0)
   (env_lv
    :reader env_lv
    :initarg :env_lv
    :type cl:float
    :initform 0.0)
   (fps
    :reader fps
    :initarg :fps
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetAe-response (<GetAe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<GetAe-response> is deprecated: use odin_ros_driver-srv:GetAe-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:success-val is deprecated.  Use odin_ros_driver-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'rc-val :lambda-list '(m))
(cl:defmethod rc-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:rc-val is deprecated.  Use odin_ros_driver-srv:rc instead.")
  (rc m))

(cl:ensure-generic-function 'exposure_time-val :lambda-list '(m))
(cl:defmethod exposure_time-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:exposure_time-val is deprecated.  Use odin_ros_driver-srv:exposure_time instead.")
  (exposure_time m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:gain-val is deprecated.  Use odin_ros_driver-srv:gain instead.")
  (gain m))

(cl:ensure-generic-function 'iso-val :lambda-list '(m))
(cl:defmethod iso-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:iso-val is deprecated.  Use odin_ros_driver-srv:iso instead.")
  (iso m))

(cl:ensure-generic-function 'brightness-val :lambda-list '(m))
(cl:defmethod brightness-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:brightness-val is deprecated.  Use odin_ros_driver-srv:brightness instead.")
  (brightness m))

(cl:ensure-generic-function 'is_converged-val :lambda-list '(m))
(cl:defmethod is_converged-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:is_converged-val is deprecated.  Use odin_ros_driver-srv:is_converged instead.")
  (is_converged m))

(cl:ensure-generic-function 'env_lv-val :lambda-list '(m))
(cl:defmethod env_lv-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:env_lv-val is deprecated.  Use odin_ros_driver-srv:env_lv instead.")
  (env_lv m))

(cl:ensure-generic-function 'fps-val :lambda-list '(m))
(cl:defmethod fps-val ((m <GetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:fps-val is deprecated.  Use odin_ros_driver-srv:fps instead.")
  (fps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAe-response>) ostream)
  "Serializes a message object of type '<GetAe-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'rc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'exposure_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'iso)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'brightness))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_converged)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'env_lv))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'fps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAe-response>) istream)
  "Deserializes a message object of type '<GetAe-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rc) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'exposure_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'iso) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'brightness) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_converged)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'env_lv) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'fps) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAe-response>)))
  "Returns string type for a service object of type '<GetAe-response>"
  "odin_ros_driver/GetAeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAe-response)))
  "Returns string type for a service object of type 'GetAe-response"
  "odin_ros_driver/GetAeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAe-response>)))
  "Returns md5sum for a message object of type '<GetAe-response>"
  "542c4f6644aaf51ceda9dea371da93c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAe-response)))
  "Returns md5sum for a message object of type 'GetAe-response"
  "542c4f6644aaf51ceda9dea371da93c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAe-response>)))
  "Returns full string definition for message of type '<GetAe-response>"
  (cl:format cl:nil "bool success            # true if rc == 0~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%float32 exposure_time   # seconds (manual range 0.0001 .. 0.033)~%float32 gain            # analog gain~%int32 iso               # equivalent ISO~%float32 brightness      # average frame brightness~%uint8 is_converged      # 1=converged, 0=not converged~%float32 env_lv          # ambient luminance level~%float32 fps             # current frame rate~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAe-response)))
  "Returns full string definition for message of type 'GetAe-response"
  (cl:format cl:nil "bool success            # true if rc == 0~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%float32 exposure_time   # seconds (manual range 0.0001 .. 0.033)~%float32 gain            # analog gain~%int32 iso               # equivalent ISO~%float32 brightness      # average frame brightness~%uint8 is_converged      # 1=converged, 0=not converged~%float32 env_lv          # ambient luminance level~%float32 fps             # current frame rate~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAe-response>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAe-response
    (cl:cons ':success (success msg))
    (cl:cons ':rc (rc msg))
    (cl:cons ':exposure_time (exposure_time msg))
    (cl:cons ':gain (gain msg))
    (cl:cons ':iso (iso msg))
    (cl:cons ':brightness (brightness msg))
    (cl:cons ':is_converged (is_converged msg))
    (cl:cons ':env_lv (env_lv msg))
    (cl:cons ':fps (fps msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAe)))
  'GetAe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAe)))
  'GetAe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAe)))
  "Returns string type for a service object of type '<GetAe>"
  "odin_ros_driver/GetAe")