; Auto-generated. Do not edit!


(cl:in-package first_project-msg)


;//! \htmlinclude secotor_times.msg.html

(cl:defclass <secotor_times> (roslisp-msg-protocol:ros-message)
  ((current_sector
    :reader current_sector
    :initarg :current_sector
    :type cl:integer
    :initform 0)
   (current_sector_time
    :reader current_sector_time
    :initarg :current_sector_time
    :type cl:float
    :initform 0.0)
   (current_sector_mean_speed
    :reader current_sector_mean_speed
    :initarg :current_sector_mean_speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass secotor_times (<secotor_times>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <secotor_times>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'secotor_times)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name first_project-msg:<secotor_times> is deprecated: use first_project-msg:secotor_times instead.")))

(cl:ensure-generic-function 'current_sector-val :lambda-list '(m))
(cl:defmethod current_sector-val ((m <secotor_times>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader first_project-msg:current_sector-val is deprecated.  Use first_project-msg:current_sector instead.")
  (current_sector m))

(cl:ensure-generic-function 'current_sector_time-val :lambda-list '(m))
(cl:defmethod current_sector_time-val ((m <secotor_times>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader first_project-msg:current_sector_time-val is deprecated.  Use first_project-msg:current_sector_time instead.")
  (current_sector_time m))

(cl:ensure-generic-function 'current_sector_mean_speed-val :lambda-list '(m))
(cl:defmethod current_sector_mean_speed-val ((m <secotor_times>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader first_project-msg:current_sector_mean_speed-val is deprecated.  Use first_project-msg:current_sector_mean_speed instead.")
  (current_sector_mean_speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <secotor_times>) ostream)
  "Serializes a message object of type '<secotor_times>"
  (cl:let* ((signed (cl:slot-value msg 'current_sector)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_sector_time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'current_sector_mean_speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <secotor_times>) istream)
  "Deserializes a message object of type '<secotor_times>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_sector) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_sector_time) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current_sector_mean_speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<secotor_times>)))
  "Returns string type for a message object of type '<secotor_times>"
  "first_project/secotor_times")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'secotor_times)))
  "Returns string type for a message object of type 'secotor_times"
  "first_project/secotor_times")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<secotor_times>)))
  "Returns md5sum for a message object of type '<secotor_times>"
  "245a0ea055f4366c2b43f7ea3395f4ec")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'secotor_times)))
  "Returns md5sum for a message object of type 'secotor_times"
  "245a0ea055f4366c2b43f7ea3395f4ec")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<secotor_times>)))
  "Returns full string definition for message of type '<secotor_times>"
  (cl:format cl:nil "int32 current_sector~%float32 current_sector_time~%float32 current_sector_mean_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'secotor_times)))
  "Returns full string definition for message of type 'secotor_times"
  (cl:format cl:nil "int32 current_sector~%float32 current_sector_time~%float32 current_sector_mean_speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <secotor_times>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <secotor_times>))
  "Converts a ROS message object to a list"
  (cl:list 'secotor_times
    (cl:cons ':current_sector (current_sector msg))
    (cl:cons ':current_sector_time (current_sector_time msg))
    (cl:cons ':current_sector_mean_speed (current_sector_mean_speed msg))
))
