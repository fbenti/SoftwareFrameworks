
(cl:in-package :asdf)

(defsystem "hello_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "turtle" :depends-on ("_package_turtle"))
    (:file "_package_turtle" :depends-on ("_package"))
  ))