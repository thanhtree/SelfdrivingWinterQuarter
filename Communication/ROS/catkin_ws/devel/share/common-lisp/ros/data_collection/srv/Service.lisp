; Auto-generated. Do not edit!


(cl:in-package data_collection-srv)


;//! \htmlinclude Service-request.msg.html

(cl:defclass <Service-request> (roslisp-msg-protocol:ros-message)
  ((num
    :reader num
    :initarg :num
    :type cl:integer
    :initform 0))
)

(cl:defclass Service-request (<Service-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Service-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Service-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_collection-srv:<Service-request> is deprecated: use data_collection-srv:Service-request instead.")))

(cl:ensure-generic-function 'num-val :lambda-list '(m))
(cl:defmethod num-val ((m <Service-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader data_collection-srv:num-val is deprecated.  Use data_collection-srv:num instead.")
  (num m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Service-request>) ostream)
  "Serializes a message object of type '<Service-request>"
  (cl:let* ((signed (cl:slot-value msg 'num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Service-request>) istream)
  "Deserializes a message object of type '<Service-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Service-request>)))
  "Returns string type for a service object of type '<Service-request>"
  "data_collection/ServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Service-request)))
  "Returns string type for a service object of type 'Service-request"
  "data_collection/ServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Service-request>)))
  "Returns md5sum for a message object of type '<Service-request>"
  "57d3c40ec3ac3754af76a83e6e73127a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Service-request)))
  "Returns md5sum for a message object of type 'Service-request"
  "57d3c40ec3ac3754af76a83e6e73127a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Service-request>)))
  "Returns full string definition for message of type '<Service-request>"
  (cl:format cl:nil "int64 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Service-request)))
  "Returns full string definition for message of type 'Service-request"
  (cl:format cl:nil "int64 num~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Service-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Service-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Service-request
    (cl:cons ':num (num msg))
))
;//! \htmlinclude Service-response.msg.html

(cl:defclass <Service-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Service-response (<Service-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Service-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Service-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name data_collection-srv:<Service-response> is deprecated: use data_collection-srv:Service-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Service-response>) ostream)
  "Serializes a message object of type '<Service-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Service-response>) istream)
  "Deserializes a message object of type '<Service-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Service-response>)))
  "Returns string type for a service object of type '<Service-response>"
  "data_collection/ServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Service-response)))
  "Returns string type for a service object of type 'Service-response"
  "data_collection/ServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Service-response>)))
  "Returns md5sum for a message object of type '<Service-response>"
  "57d3c40ec3ac3754af76a83e6e73127a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Service-response)))
  "Returns md5sum for a message object of type 'Service-response"
  "57d3c40ec3ac3754af76a83e6e73127a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Service-response>)))
  "Returns full string definition for message of type '<Service-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Service-response)))
  "Returns full string definition for message of type 'Service-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Service-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Service-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Service-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Service)))
  'Service-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Service)))
  'Service-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Service)))
  "Returns string type for a service object of type '<Service>"
  "data_collection/Service")