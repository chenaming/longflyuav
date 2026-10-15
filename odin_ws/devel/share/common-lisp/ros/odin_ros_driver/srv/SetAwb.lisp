; Auto-generated. Do not edit!


(cl:in-package odin_ros_driver-srv)


;//! \htmlinclude SetAwb-request.msg.html

(cl:defclass <SetAwb-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (rgain
    :reader rgain
    :initarg :rgain
    :type cl:float
    :initform 0.0)
   (bgain
    :reader bgain
    :initarg :bgain
    :type cl:float
    :initform 0.0))
)

(cl:defclass SetAwb-request (<SetAwb-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAwb-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAwb-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<SetAwb-request> is deprecated: use odin_ros_driver-srv:SetAwb-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetAwb-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:mode-val is deprecated.  Use odin_ros_driver-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'rgain-val :lambda-list '(m))
(cl:defmethod rgain-val ((m <SetAwb-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:rgain-val is deprecated.  Use odin_ros_driver-srv:rgain instead.")
  (rgain m))

(cl:ensure-generic-function 'bgain-val :lambda-list '(m))
(cl:defmethod bgain-val ((m <SetAwb-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:bgain-val is deprecated.  Use odin_ros_driver-srv:bgain instead.")
  (bgain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAwb-request>) ostream)
  "Serializes a message object of type '<SetAwb-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rgain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bgain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAwb-request>) istream)
  "Deserializes a message object of type '<SetAwb-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'mode)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rgain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bgain) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAwb-request>)))
  "Returns string type for a service object of type '<SetAwb-request>"
  "odin_ros_driver/SetAwbRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAwb-request)))
  "Returns string type for a service object of type 'SetAwb-request"
  "odin_ros_driver/SetAwbRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAwb-request>)))
  "Returns md5sum for a message object of type '<SetAwb-request>"
  "b1976cc70ace03d84cebe2d1a9bc817e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAwb-request)))
  "Returns md5sum for a message object of type 'SetAwb-request"
  "b1976cc70ace03d84cebe2d1a9bc817e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAwb-request>)))
  "Returns full string definition for message of type '<SetAwb-request>"
  (cl:format cl:nil "# Set AWB (auto white balance) mode and (in manual mode) R/B gains.~%#~%# mode == 0 (AUTO)   : sends opcode 0x31 only; rgain and bgain ignored.~%# mode == 1 (MANUAL) : sends opcode 0x32 then 0x33 (rgain, bgain).~%#                     Gr/Gb are fixed to 1.0 by the device.~%#~%# Valid manual ranges:~%#   rgain : 0.1 .. 4.0~%#   bgain : 0.1 .. 4.0~%uint8 mode~%float32 rgain~%float32 bgain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAwb-request)))
  "Returns full string definition for message of type 'SetAwb-request"
  (cl:format cl:nil "# Set AWB (auto white balance) mode and (in manual mode) R/B gains.~%#~%# mode == 0 (AUTO)   : sends opcode 0x31 only; rgain and bgain ignored.~%# mode == 1 (MANUAL) : sends opcode 0x32 then 0x33 (rgain, bgain).~%#                     Gr/Gb are fixed to 1.0 by the device.~%#~%# Valid manual ranges:~%#   rgain : 0.1 .. 4.0~%#   bgain : 0.1 .. 4.0~%uint8 mode~%float32 rgain~%float32 bgain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAwb-request>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAwb-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAwb-request
    (cl:cons ':mode (mode msg))
    (cl:cons ':rgain (rgain msg))
    (cl:cons ':bgain (bgain msg))
))
;//! \htmlinclude SetAwb-response.msg.html

(cl:defclass <SetAwb-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass SetAwb-response (<SetAwb-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetAwb-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetAwb-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<SetAwb-response> is deprecated: use odin_ros_driver-srv:SetAwb-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:success-val is deprecated.  Use odin_ros_driver-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'rc-val :lambda-list '(m))
(cl:defmethod rc-val ((m <SetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:rc-val is deprecated.  Use odin_ros_driver-srv:rc instead.")
  (rc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetAwb-response>) ostream)
  "Serializes a message object of type '<SetAwb-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'rc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetAwb-response>) istream)
  "Deserializes a message object of type '<SetAwb-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rc) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetAwb-response>)))
  "Returns string type for a service object of type '<SetAwb-response>"
  "odin_ros_driver/SetAwbResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAwb-response)))
  "Returns string type for a service object of type 'SetAwb-response"
  "odin_ros_driver/SetAwbResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetAwb-response>)))
  "Returns md5sum for a message object of type '<SetAwb-response>"
  "b1976cc70ace03d84cebe2d1a9bc817e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetAwb-response)))
  "Returns md5sum for a message object of type 'SetAwb-response"
  "b1976cc70ace03d84cebe2d1a9bc817e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetAwb-response>)))
  "Returns full string definition for message of type '<SetAwb-response>"
  (cl:format cl:nil "bool success~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetAwb-response)))
  "Returns full string definition for message of type 'SetAwb-response"
  (cl:format cl:nil "bool success~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetAwb-response>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetAwb-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetAwb-response
    (cl:cons ':success (success msg))
    (cl:cons ':rc (rc msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetAwb)))
  'SetAwb-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetAwb)))
  'SetAwb-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetAwb)))
  "Returns string type for a service object of type '<SetAwb>"
  "odin_ros_driver/SetAwb")