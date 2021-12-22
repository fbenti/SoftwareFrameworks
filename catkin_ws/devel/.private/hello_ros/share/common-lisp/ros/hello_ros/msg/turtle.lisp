; Auto-generated. Do not edit!


(cl:in-package hello_ros-msg)


;//! \htmlinclude turtle.msg.html

(cl:defclass <turtle> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass turtle (<turtle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <turtle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'turtle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hello_ros-msg:<turtle> is deprecated: use hello_ros-msg:turtle instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <turtle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hello_ros-msg:name-val is deprecated.  Use hello_ros-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <turtle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hello_ros-msg:speed-val is deprecated.  Use hello_ros-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <turtle>) ostream)
  "Serializes a message object of type '<turtle>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <turtle>) istream)
  "Deserializes a message object of type '<turtle>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<turtle>)))
  "Returns string type for a message object of type '<turtle>"
  "hello_ros/turtle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'turtle)))
  "Returns string type for a message object of type 'turtle"
  "hello_ros/turtle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<turtle>)))
  "Returns md5sum for a message object of type '<turtle>"
  "731354329381c2f16be8e48fe990ad74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'turtle)))
  "Returns md5sum for a message object of type 'turtle"
  "731354329381c2f16be8e48fe990ad74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<turtle>)))
  "Returns full string definition for message of type '<turtle>"
  (cl:format cl:nil "string name~%float32 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'turtle)))
  "Returns full string definition for message of type 'turtle"
  (cl:format cl:nil "string name~%float32 speed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <turtle>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <turtle>))
  "Converts a ROS message object to a list"
  (cl:list 'turtle
    (cl:cons ':name (name msg))
    (cl:cons ':speed (speed msg))
))
