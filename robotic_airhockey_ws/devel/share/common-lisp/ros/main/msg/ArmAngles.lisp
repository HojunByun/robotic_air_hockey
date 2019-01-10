; Auto-generated. Do not edit!


(cl:in-package main-msg)


;//! \htmlinclude ArmAngles.msg.html

(cl:defclass <ArmAngles> (roslisp-msg-protocol:ros-message)
  ((arm0_joint0
    :reader arm0_joint0
    :initarg :arm0_joint0
    :type cl:float
    :initform 0.0)
   (arm0_joint1
    :reader arm0_joint1
    :initarg :arm0_joint1
    :type cl:float
    :initform 0.0)
   (arm1_joint0
    :reader arm1_joint0
    :initarg :arm1_joint0
    :type cl:float
    :initform 0.0)
   (arm1_joint1
    :reader arm1_joint1
    :initarg :arm1_joint1
    :type cl:float
    :initform 0.0)
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ArmAngles (<ArmAngles>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ArmAngles>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ArmAngles)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name main-msg:<ArmAngles> is deprecated: use main-msg:ArmAngles instead.")))

(cl:ensure-generic-function 'arm0_joint0-val :lambda-list '(m))
(cl:defmethod arm0_joint0-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main-msg:arm0_joint0-val is deprecated.  Use main-msg:arm0_joint0 instead.")
  (arm0_joint0 m))

(cl:ensure-generic-function 'arm0_joint1-val :lambda-list '(m))
(cl:defmethod arm0_joint1-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main-msg:arm0_joint1-val is deprecated.  Use main-msg:arm0_joint1 instead.")
  (arm0_joint1 m))

(cl:ensure-generic-function 'arm1_joint0-val :lambda-list '(m))
(cl:defmethod arm1_joint0-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main-msg:arm1_joint0-val is deprecated.  Use main-msg:arm1_joint0 instead.")
  (arm1_joint0 m))

(cl:ensure-generic-function 'arm1_joint1-val :lambda-list '(m))
(cl:defmethod arm1_joint1-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main-msg:arm1_joint1-val is deprecated.  Use main-msg:arm1_joint1 instead.")
  (arm1_joint1 m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ArmAngles>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader main-msg:success-val is deprecated.  Use main-msg:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ArmAngles>) ostream)
  "Serializes a message object of type '<ArmAngles>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'arm0_joint0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'arm0_joint1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'arm1_joint0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'arm1_joint1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ArmAngles>) istream)
  "Deserializes a message object of type '<ArmAngles>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm0_joint0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm0_joint1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm1_joint0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'arm1_joint1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ArmAngles>)))
  "Returns string type for a message object of type '<ArmAngles>"
  "main/ArmAngles")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ArmAngles)))
  "Returns string type for a message object of type 'ArmAngles"
  "main/ArmAngles")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ArmAngles>)))
  "Returns md5sum for a message object of type '<ArmAngles>"
  "03c786ea5bb5b9ff9771f16c1a88ee81")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ArmAngles)))
  "Returns md5sum for a message object of type 'ArmAngles"
  "03c786ea5bb5b9ff9771f16c1a88ee81")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ArmAngles>)))
  "Returns full string definition for message of type '<ArmAngles>"
  (cl:format cl:nil "float32 arm0_joint0~%float32 arm0_joint1~%float32 arm1_joint0~%float32 arm1_joint1~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ArmAngles)))
  "Returns full string definition for message of type 'ArmAngles"
  (cl:format cl:nil "float32 arm0_joint0~%float32 arm0_joint1~%float32 arm1_joint0~%float32 arm1_joint1~%bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ArmAngles>))
  (cl:+ 0
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ArmAngles>))
  "Converts a ROS message object to a list"
  (cl:list 'ArmAngles
    (cl:cons ':arm0_joint0 (arm0_joint0 msg))
    (cl:cons ':arm0_joint1 (arm0_joint1 msg))
    (cl:cons ':arm1_joint0 (arm1_joint0 msg))
    (cl:cons ':arm1_joint1 (arm1_joint1 msg))
    (cl:cons ':success (success msg))
))
