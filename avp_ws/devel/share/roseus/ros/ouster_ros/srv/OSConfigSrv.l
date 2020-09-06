;; Auto-generated. Do not edit!


(when (boundp 'ouster_ros::OSConfigSrv)
  (if (not (find-package "OUSTER_ROS"))
    (make-package "OUSTER_ROS"))
  (shadow 'OSConfigSrv (find-package "OUSTER_ROS")))
(unless (find-package "OUSTER_ROS::OSCONFIGSRV")
  (make-package "OUSTER_ROS::OSCONFIGSRV"))
(unless (find-package "OUSTER_ROS::OSCONFIGSRVREQUEST")
  (make-package "OUSTER_ROS::OSCONFIGSRVREQUEST"))
(unless (find-package "OUSTER_ROS::OSCONFIGSRVRESPONSE")
  (make-package "OUSTER_ROS::OSCONFIGSRVRESPONSE"))

(in-package "ROS")





(defclass ouster_ros::OSConfigSrvRequest
  :super ros::object
  :slots ())

(defmethod ouster_ros::OSConfigSrvRequest
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

(defclass ouster_ros::OSConfigSrvResponse
  :super ros::object
  :slots (_metadata ))

(defmethod ouster_ros::OSConfigSrvResponse
  (:init
   (&key
    ((:metadata __metadata) "")
    )
   (send-super :init)
   (setq _metadata (string __metadata))
   self)
  (:metadata
   (&optional __metadata)
   (if __metadata (setq _metadata __metadata)) _metadata)
  (:serialization-length
   ()
   (+
    ;; string _metadata
    4 (length _metadata)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _metadata
       (write-long (length _metadata) s) (princ _metadata s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _metadata
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _metadata (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ouster_ros::OSConfigSrv
  :super ros::object
  :slots ())

(setf (get ouster_ros::OSConfigSrv :md5sum-) "d37888e5a47bef783c189dec5351027e")
(setf (get ouster_ros::OSConfigSrv :datatype-) "ouster_ros/OSConfigSrv")
(setf (get ouster_ros::OSConfigSrv :request) ouster_ros::OSConfigSrvRequest)
(setf (get ouster_ros::OSConfigSrv :response) ouster_ros::OSConfigSrvResponse)

(defmethod ouster_ros::OSConfigSrvRequest
  (:response () (instance ouster_ros::OSConfigSrvResponse :init)))

(setf (get ouster_ros::OSConfigSrvRequest :md5sum-) "d37888e5a47bef783c189dec5351027e")
(setf (get ouster_ros::OSConfigSrvRequest :datatype-) "ouster_ros/OSConfigSrvRequest")
(setf (get ouster_ros::OSConfigSrvRequest :definition-)
      "---
string metadata

")

(setf (get ouster_ros::OSConfigSrvResponse :md5sum-) "d37888e5a47bef783c189dec5351027e")
(setf (get ouster_ros::OSConfigSrvResponse :datatype-) "ouster_ros/OSConfigSrvResponse")
(setf (get ouster_ros::OSConfigSrvResponse :definition-)
      "---
string metadata

")



(provide :ouster_ros/OSConfigSrv "d37888e5a47bef783c189dec5351027e")


