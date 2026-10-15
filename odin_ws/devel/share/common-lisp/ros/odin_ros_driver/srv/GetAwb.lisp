; Auto-generated. Do not edit!


(cl:in-package odin_ros_driver-srv)


;//! \htmlinclude GetAwb-request.msg.html

(cl:defclass <GetAwb-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetAwb-request (<GetAwb-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAwb-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAwb-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<GetAwb-request> is deprecated: use odin_ros_driver-srv:GetAwb-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAwb-request>) ostream)
  "Serializes a message object of type '<GetAwb-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAwb-request>) istream)
  "Deserializes a message object of type '<GetAwb-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAwb-request>)))
  "Returns string type for a service object of type '<GetAwb-request>"
  "odin_ros_driver/GetAwbRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAwb-request)))
  "Returns string type for a service object of type 'GetAwb-request"
  "odin_ros_driver/GetAwbRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAwb-request>)))
  "Returns md5sum for a message object of type '<GetAwb-request>"
  "508f4cd8169f95ac5caf4673a24ce5b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAwb-request)))
  "Returns md5sum for a message object of type 'GetAwb-request"
  "508f4cd8169f95ac5caf4673a24ce5b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAwb-request>)))
  "Returns full string definition for message of type '<GetAwb-request>"
  (cl:format cl:nil "# Query current AWB (auto white balance) state.~%# Request: empty.~%# Response: success + raw return code + decoded fields.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAwb-request)))
  "Returns full string definition for message of type 'GetAwb-request"
  (cl:format cl:nil "# Query current AWB (auto white balance) state.~%# Request: empty.~%# Response: success + raw return code + decoded fields.~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAwb-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAwb-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAwb-request
))
;//! \htmlinclude GetAwb-response.msg.html

(cl:defclass <GetAwb-response> (roslisp-msg-protocol:ros-message)
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
   (rgain
    :reader rgain
    :initarg :rgain
    :type cl:float
    :initform 0.0)
   (grgain
    :reader grgain
    :initarg :grgain
    :type cl:float
    :initform 0.0)
   (gbgain
    :reader gbgain
    :initarg :gbgain
    :type cl:float
    :initform 0.0)
   (bgain
    :reader bgain
    :initarg :bgain
    :type cl:float
    :initform 0.0)
   (cct
    :reader cct
    :initarg :cct
    :type cl:float
    :initform 0.0)
   (ccri
    :reader ccri
    :initarg :ccri
    :type cl:float
    :initform 0.0)
   (is_converged
    :reader is_converged
    :initarg :is_converged
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GetAwb-response (<GetAwb-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetAwb-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetAwb-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name odin_ros_driver-srv:<GetAwb-response> is deprecated: use odin_ros_driver-srv:GetAwb-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:success-val is deprecated.  Use odin_ros_driver-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'rc-val :lambda-list '(m))
(cl:defmethod rc-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:rc-val is deprecated.  Use odin_ros_driver-srv:rc instead.")
  (rc m))

(cl:ensure-generic-function 'rgain-val :lambda-list '(m))
(cl:defmethod rgain-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:rgain-val is deprecated.  Use odin_ros_driver-srv:rgain instead.")
  (rgain m))

(cl:ensure-generic-function 'grgain-val :lambda-list '(m))
(cl:defmethod grgain-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:grgain-val is deprecated.  Use odin_ros_driver-srv:grgain instead.")
  (grgain m))

(cl:ensure-generic-function 'gbgain-val :lambda-list '(m))
(cl:defmethod gbgain-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:gbgain-val is deprecated.  Use odin_ros_driver-srv:gbgain instead.")
  (gbgain m))

(cl:ensure-generic-function 'bgain-val :lambda-list '(m))
(cl:defmethod bgain-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:bgain-val is deprecated.  Use odin_ros_driver-srv:bgain instead.")
  (bgain m))

(cl:ensure-generic-function 'cct-val :lambda-list '(m))
(cl:defmethod cct-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:cct-val is deprecated.  Use odin_ros_driver-srv:cct instead.")
  (cct m))

(cl:ensure-generic-function 'ccri-val :lambda-list '(m))
(cl:defmethod ccri-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:ccri-val is deprecated.  Use odin_ros_driver-srv:ccri instead.")
  (ccri m))

(cl:ensure-generic-function 'is_converged-val :lambda-list '(m))
(cl:defmethod is_converged-val ((m <GetAwb-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader odin_ros_driver-srv:is_converged-val is deprecated.  Use odin_ros_driver-srv:is_converged instead.")
  (is_converged m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetAwb-response>) ostream)
  "Serializes a message object of type '<GetAwb-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'rc)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rgain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'grgain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gbgain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bgain))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cct))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ccri))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_converged)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetAwb-response>) istream)
  "Deserializes a message object of type '<GetAwb-response>"
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
    (cl:setf (cl:slot-value msg 'rgain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'grgain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gbgain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bgain) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cct) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ccri) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'is_converged)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetAwb-response>)))
  "Returns string type for a service object of type '<GetAwb-response>"
  "odin_ros_driver/GetAwbResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAwb-response)))
  "Returns string type for a service object of type 'GetAwb-response"
  "odin_ros_driver/GetAwbResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetAwb-response>)))
  "Returns md5sum for a message object of type '<GetAwb-response>"
  "508f4cd8169f95ac5caf4673a24ce5b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetAwb-response)))
  "Returns md5sum for a message object of type 'GetAwb-response"
  "508f4cd8169f95ac5caf4673a24ce5b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetAwb-response>)))
  "Returns full string definition for message of type '<GetAwb-response>"
  (cl:format cl:nil "bool success            # true if rc == 0~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%float32 rgain~%float32 grgain~%float32 gbgain~%float32 bgain~%float32 cct             # color temperature in Kelvin~%float32 ccri            # color temperature deviation~%uint8 is_converged      # 1=converged, 0=not converged~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetAwb-response)))
  "Returns full string definition for message of type 'GetAwb-response"
  (cl:format cl:nil "bool success            # true if rc == 0~%int32 rc                # 0=ok, >0=device error (400..405 or 0xFF), <0=SDK error~%float32 rgain~%float32 grgain~%float32 gbgain~%float32 bgain~%float32 cct             # color temperature in Kelvin~%float32 ccri            # color temperature deviation~%uint8 is_converged      # 1=converged, 0=not converged~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetAwb-response>))
  (cl:+ 0
     1
     4
     4
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetAwb-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetAwb-response
    (cl:cons ':success (success msg))
    (cl:cons ':rc (rc msg))
    (cl:cons ':rgain (rgain msg))
    (cl:cons ':grgain (grgain msg))
    (cl:cons ':gbgain (gbgain msg))
    (cl:cons ':bgain (bgain msg))
    (cl:cons ':cct (cct msg))
    (cl:cons ':ccri (ccri msg))
    (cl:cons ':is_converged (is_converged msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetAwb)))
  'GetAwb-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetAwb)))
  'GetAwb-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetAwb)))
  "Returns string type for a service object of type '<GetAwb>"
  "odin_ros_driver/GetAwb")