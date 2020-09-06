;; Auto-generated. Do not edit!


(when (boundp 'ouster_ros::OS1ConfigSrv)
  (if (not (find-package "OUSTER_ROS"))
    (make-package "OUSTER_ROS"))
  (shadow 'OS1ConfigSrv (find-package "OUSTER_ROS")))
(unless (find-package "OUSTER_ROS::OS1CONFIGSRV")
  (make-package "OUSTER_ROS::OS1CONFIGSRV"))
(unless (find-package "OUSTER_ROS::OS1CONFIGSRVREQUEST")
  (make-package "OUSTER_ROS::OS1CONFIGSRVREQUEST"))
(unless (find-package "OUSTER_ROS::OS1CONFIGSRVRESPONSE")
  (make-package "OUSTER_ROS::OS1CONFIGSRVRESPONSE"))

(in-package "ROS")





(defclass ouster_ros::OS1ConfigSrvRequest
  :super ros::object
  :slots ())

(defmethod ouster_ros::OS1ConfigSrvRequest
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

(defclass ouster_ros::OS1ConfigSrvResponse
  :super ros::object
  :slots (_hostname _lidar_mode _beam_azimuth_angles _beam_altitude_angles _imu_to_sensor_transform _lidar_to_sensor_transform ))

(defmethod ouster_ros::OS1ConfigSrvResponse
  (:init
   (&key
    ((:hostname __hostname) "")
    ((:lidar_mode __lidar_mode) "")
    ((:beam_azimuth_angles __beam_azimuth_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:beam_altitude_angles __beam_altitude_angles) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:imu_to_sensor_transform __imu_to_sensor_transform) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:lidar_to_sensor_transform __lidar_to_sensor_transform) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _hostname (string __hostname))
   (setq _lidar_mode (string __lidar_mode))
   (setq _beam_azimuth_angles __beam_azimuth_angles)
   (setq _beam_altitude_angles __beam_altitude_angles)
   (setq _imu_to_sensor_transform __imu_to_sensor_transform)
   (setq _lidar_to_sensor_transform __lidar_to_sensor_transform)
   self)
  (:hostname
   (&optional __hostname)
   (if __hostname (setq _hostname __hostname)) _hostname)
  (:lidar_mode
   (&optional __lidar_mode)
   (if __lidar_mode (setq _lidar_mode __lidar_mode)) _lidar_mode)
  (:beam_azimuth_angles
   (&optional __beam_azimuth_angles)
   (if __beam_azimuth_angles (setq _beam_azimuth_angles __beam_azimuth_angles)) _beam_azimuth_angles)
  (:beam_altitude_angles
   (&optional __beam_altitude_angles)
   (if __beam_altitude_angles (setq _beam_altitude_angles __beam_altitude_angles)) _beam_altitude_angles)
  (:imu_to_sensor_transform
   (&optional __imu_to_sensor_transform)
   (if __imu_to_sensor_transform (setq _imu_to_sensor_transform __imu_to_sensor_transform)) _imu_to_sensor_transform)
  (:lidar_to_sensor_transform
   (&optional __lidar_to_sensor_transform)
   (if __lidar_to_sensor_transform (setq _lidar_to_sensor_transform __lidar_to_sensor_transform)) _lidar_to_sensor_transform)
  (:serialization-length
   ()
   (+
    ;; string _hostname
    4 (length _hostname)
    ;; string _lidar_mode
    4 (length _lidar_mode)
    ;; float64[] _beam_azimuth_angles
    (* 8    (length _beam_azimuth_angles)) 4
    ;; float64[] _beam_altitude_angles
    (* 8    (length _beam_altitude_angles)) 4
    ;; float64[] _imu_to_sensor_transform
    (* 8    (length _imu_to_sensor_transform)) 4
    ;; float64[] _lidar_to_sensor_transform
    (* 8    (length _lidar_to_sensor_transform)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _hostname
       (write-long (length _hostname) s) (princ _hostname s)
     ;; string _lidar_mode
       (write-long (length _lidar_mode) s) (princ _lidar_mode s)
     ;; float64[] _beam_azimuth_angles
     (write-long (length _beam_azimuth_angles) s)
     (dotimes (i (length _beam_azimuth_angles))
       (sys::poke (elt _beam_azimuth_angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _beam_altitude_angles
     (write-long (length _beam_altitude_angles) s)
     (dotimes (i (length _beam_altitude_angles))
       (sys::poke (elt _beam_altitude_angles i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _imu_to_sensor_transform
     (write-long (length _imu_to_sensor_transform) s)
     (dotimes (i (length _imu_to_sensor_transform))
       (sys::poke (elt _imu_to_sensor_transform i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _lidar_to_sensor_transform
     (write-long (length _lidar_to_sensor_transform) s)
     (dotimes (i (length _lidar_to_sensor_transform))
       (sys::poke (elt _lidar_to_sensor_transform i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _hostname
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _hostname (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _lidar_mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _lidar_mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _beam_azimuth_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _beam_azimuth_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _beam_azimuth_angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _beam_altitude_angles
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _beam_altitude_angles (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _beam_altitude_angles i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _imu_to_sensor_transform
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _imu_to_sensor_transform (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _imu_to_sensor_transform i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _lidar_to_sensor_transform
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lidar_to_sensor_transform (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _lidar_to_sensor_transform i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass ouster_ros::OS1ConfigSrv
  :super ros::object
  :slots ())

(setf (get ouster_ros::OS1ConfigSrv :md5sum-) "bb865a85fb3f39f43bf99730f10225b5")
(setf (get ouster_ros::OS1ConfigSrv :datatype-) "ouster_ros/OS1ConfigSrv")
(setf (get ouster_ros::OS1ConfigSrv :request) ouster_ros::OS1ConfigSrvRequest)
(setf (get ouster_ros::OS1ConfigSrv :response) ouster_ros::OS1ConfigSrvResponse)

(defmethod ouster_ros::OS1ConfigSrvRequest
  (:response () (instance ouster_ros::OS1ConfigSrvResponse :init)))

(setf (get ouster_ros::OS1ConfigSrvRequest :md5sum-) "bb865a85fb3f39f43bf99730f10225b5")
(setf (get ouster_ros::OS1ConfigSrvRequest :datatype-) "ouster_ros/OS1ConfigSrvRequest")
(setf (get ouster_ros::OS1ConfigSrvRequest :definition-)
      "---
string hostname
string lidar_mode
float64[] beam_azimuth_angles
float64[] beam_altitude_angles
float64[] imu_to_sensor_transform
float64[] lidar_to_sensor_transform
")

(setf (get ouster_ros::OS1ConfigSrvResponse :md5sum-) "bb865a85fb3f39f43bf99730f10225b5")
(setf (get ouster_ros::OS1ConfigSrvResponse :datatype-) "ouster_ros/OS1ConfigSrvResponse")
(setf (get ouster_ros::OS1ConfigSrvResponse :definition-)
      "---
string hostname
string lidar_mode
float64[] beam_azimuth_angles
float64[] beam_altitude_angles
float64[] imu_to_sensor_transform
float64[] lidar_to_sensor_transform
")



(provide :ouster_ros/OS1ConfigSrv "bb865a85fb3f39f43bf99730f10225b5")


