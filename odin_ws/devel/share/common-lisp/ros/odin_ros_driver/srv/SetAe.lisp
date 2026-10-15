; Auto-generated. Do not edit!


(cl:in-package odin_ros_driver-srv)


;//! \htmlinclude SetAe-request.msg.html

(cl:defclass <SetAe-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
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
    :initform 0.0))
)

(cl:defclass SetAe-request (<SetAe-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAe-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAe-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<SetAe-request> is deprecated: use odin_ros_driver-srv:SetAe-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetAe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:mode-val is deprecated.  Use odin_ros_driver-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'exposure_time-val :lambda-list '(m))
(cl:defmethod exposure_time-val ((m <SetAe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:exposure_time-val is deprecated.  Use odin_ros_driver-srv:exposure_time instead.")
  (exposure_time m))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <SetAe-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:gain-val is deprecated.  Use odin_ros_driver-srv:gain instead.")
  (gain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAe-request>) ostream)
  "Serializes a message object of type '<SetAe-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAe-request>) istream)
  "Deserializes a message object of type '<SetAe-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAe-request>)))
  "Returns string type for a service object of type '<SetAe-request>"
  "odin_ros_driver/SetAeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAe-request)))
  "Returns string type for a service object of type 'SetAe-request"
  "odin_ros_driver/SetAeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAe-request>)))
  "Returns md5sum for a message object of type '<SetAe-request>"
  "a79a03de733d9c62ec6c16126efd1953")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAe-request)))
  "Returns md5sum for a message object of type 'SetAe-request"
  "a79a03de733d9c62ec6c16126efd1953")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAe-request>)))
  "Returns full string definition for message of type '<SetAe-request>"
  (cl:format cl:nil "# Set AE (auto exposure) mode and (in manual mode) exposure / gain.~%#~%# mode == 0 (AUTO)   : sends opcode 0x02 only; exposure_time and gain ignored.~%# mode == 1 (MANUAL) : sends opcode 0x03 then 0x06 (exposure_time, gain).~%#~%# Valid manual ranges (device-enforced; out-of-range returns rc=403):~%#   exposure_time : 0.0001 s .. 0.033 s~%#   gain          : 1.0     .. 64.0~%uint8 mode~%float32 exposure_time~%float32 gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAe-request)))
  "Returns full string definition for message of type 'SetAe-request"
  (cl:format cl:nil "# Set AE (auto exposure) mode and (in manual mode) exposure / gain.~%#~%# mode == 0 (AUTO)   : sends opcode 0x02 only; exposure_time and gain ignored.~%# mode == 1 (MANUAL) : sends opcode 0x03 then 0x06 (exposure_time, gain).~%#~%# Valid manual ranges (device-enforced; out-of-range returns rc=403):~%#   exposure_time : 0.0001 s .. 0.033 s~%#   gain          : 1.0     .. 64.0~%uint8 mode~%float32 exposure_time~%float32 gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAe-request>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAe-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAe-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':exposure_time (exposure_time msg))
    (cl:cons ':gain (gain msg))
))
;//! \htmlinclude SetAe-response.msg.html

(cl:defclass <SetAe-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (rc
    :reader rc
    :initarg :rc
    :type cl:integer
    :initform 0))
)

(cl:defclass SetAe-response (<SetAe-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAe-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAe-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<SetAe-response> is deprecated: use odin_ros_driver-srv:SetAe-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:success-val is deprecated.  Use odin_ros_driver-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'rc-val :lambda-list '(m))
(cl:defmethod rc-val ((m <SetAe-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:rc-val is deprecated.  Use odin_ros_driver-srv:rc instead.")
  (rc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAe-response>) ostream)
  "Serializes a message object of type '<SetAe-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'rc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAe-response>) istream)
  "Deserializes a message object of type '<SetAe-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rc) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAe-response>)))
  "Returns string type for a service object of type '<SetAe-response>"
  "odin_ros_driver/SetAeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAe-response)))
  "Returns string type for a service object of type 'SetAe-response"
  "odin_ros_driver/SetAeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAe-response>)))
  "Returns md5sum for a message object of type '<SetAe-response>"
  "a79a03de733d9c62ec6c16126efd1953")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAe-response)))
  "Returns md5sum for a message object of type 'SetAe-response"
  "a79a03de733d9c62ec6c16126efd1953")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAe-response>)))
  "Returns full string definition for message of type '<SetAe-response>"
  (cl:format cl:nil "bool success~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAe-response)))
  "Returns full string definition for message of type 'SetAe-response"
  (cl:format cl:nil "bool success~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAe-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAe-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAe-response
    (cl:cons ':success (success msg))
    (cl:cons ':rc (rc msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAe)))
  'SetAe-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAe)))
  'SetAe-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAe)))
  "Returns string type for a service object of type '<SetAe>"
  "odin_ros_driver/SetAe")