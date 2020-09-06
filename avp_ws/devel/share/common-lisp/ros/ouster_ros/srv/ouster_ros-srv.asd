
(cl:in-package :asdf)

(defsystem "ouster_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OS1ConfigSrv" :depends-on ("_package_OS1ConfigSrv"))
    (:file "_package_OS1ConfigSrv" :depends-on ("_package"))
  ))