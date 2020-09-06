
(cl:in-package :asdf)

(defsystem "ouster_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "OSConfigSrv" :depends-on ("_package_OSConfigSrv"))
    (:file "_package_OSConfigSrv" :depends-on ("_package"))
  ))