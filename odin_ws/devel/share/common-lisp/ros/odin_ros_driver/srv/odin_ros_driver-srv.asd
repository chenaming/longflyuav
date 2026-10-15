
(cl:in-package :asdf)

(defsystem "odin_ros_driver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetAe" :depends-on ("_package_GetAe"))
    (:file "_package_GetAe" :depends-on ("_package"))
    (:file "GetAwb" :depends-on ("_package_GetAwb"))
    (:file "_package_GetAwb" :depends-on ("_package"))
    (:file "SetAe" :depends-on ("_package_SetAe"))
    (:file "_package_SetAe" :depends-on ("_package"))
    (:file "SetAwb" :depends-on ("_package_SetAwb"))
    (:file "_package_SetAwb" :depends-on ("_package"))
  ))