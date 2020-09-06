; Auto-generated. Do not edit!


(cl:in-package ouster_ros-srv)


;//! \htmlinclude OSConfigSrv-request.msg.html

(cl:defclass <OSConfigSrv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass OSConfigSrv-request (<OSConfigSrv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OSConfigSrv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OSConfigSrv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<OSConfigSrv-request> is deprecated: use ouster_ros-srv:OSConfigSrv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OSConfigSrv-request>) ostream)
  "Serializes a message object of type '<OSConfigSrv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OSConfigSrv-request>) istream)
  "Deserializes a message object of type '<OSConfigSrv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OSConfigSrv-request>)))
  "Returns string type for a service object of type '<OSConfigSrv-request>"
  "ouster_ros/OSConfigSrvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OSConfigSrv-request)))
  "Returns string type for a service object of type 'OSConfigSrv-request"
  "ouster_ros/OSConfigSrvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OSConfigSrv-request>)))
  "Returns md5sum for a message object of type '<OSConfigSrv-request>"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OSConfigSrv-request)))
  "Returns md5sum for a message object of type 'OSConfigSrv-request"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OSConfigSrv-request>)))
  "Returns full string definition for message of type '<OSConfigSrv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OSConfigSrv-request)))
  "Returns full string definition for message of type 'OSConfigSrv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OSConfigSrv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OSConfigSrv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'OSConfigSrv-request
))
;//! \htmlinclude OSConfigSrv-response.msg.html

(cl:defclass <OSConfigSrv-response> (roslisp-msg-protocol:ros-message)
  ((metadata
    :reader metadata
    :initarg :metadata
    :type cl:string
    :initform ""))
)

(cl:defclass OSConfigSrv-response (<OSConfigSrv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OSConfigSrv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OSConfigSrv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<OSConfigSrv-response> is deprecated: use ouster_ros-srv:OSConfigSrv-response instead.")))

(cl:ensure-generic-function 'metadata-val :lambda-list '(m))
(cl:defmethod metadata-val ((m <OSConfigSrv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ouster_ros-srv:metadata-val is deprecated.  Use ouster_ros-srv:metadata instead.")
  (metadata m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OSConfigSrv-response>) ostream)
  "Serializes a message object of type '<OSConfigSrv-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'metadata))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'metadata))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OSConfigSrv-response>) istream)
  "Deserializes a message object of type '<OSConfigSrv-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'metadata) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'metadata) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OSConfigSrv-response>)))
  "Returns string type for a service object of type '<OSConfigSrv-response>"
  "ouster_ros/OSConfigSrvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OSConfigSrv-response)))
  "Returns string type for a service object of type 'OSConfigSrv-response"
  "ouster_ros/OSConfigSrvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OSConfigSrv-response>)))
  "Returns md5sum for a message object of type '<OSConfigSrv-response>"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OSConfigSrv-response)))
  "Returns md5sum for a message object of type 'OSConfigSrv-response"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OSConfigSrv-response>)))
  "Returns full string definition for message of type '<OSConfigSrv-response>"
  (cl:format cl:nil "string metadata~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OSConfigSrv-response)))
  "Returns full string definition for message of type 'OSConfigSrv-response"
  (cl:format cl:nil "string metadata~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OSConfigSrv-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'metadata))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OSConfigSrv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'OSConfigSrv-response
    (cl:cons ':metadata (metadata msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'OSConfigSrv)))
  'OSConfigSrv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'OSConfigSrv)))
  'OSConfigSrv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OSConfigSrv)))
  "Returns string type for a service object of type '<OSConfigSrv>"
  "ouster_ros/OSConfigSrv")