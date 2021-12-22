;; Auto-generated. Do not edit!


(when (boundp 'hello_ros::turtle)
  (if (not (find-package "HELLO_ROS"))
    (make-package "HELLO_ROS"))
  (shadow 'turtle (find-package "HELLO_ROS")))
(unless (find-package "HELLO_ROS::TURTLE")
  (make-package "HELLO_ROS::TURTLE"))

(in-package "ROS")
;;//! \htmlinclude turtle.msg.html


(defclass hello_ros::turtle
  :super ros::object
  :slots (_name _speed ))

(defmethod hello_ros::turtle
  (:init
   (&key
    ((:name __name) "")
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _speed (float __speed))
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hello_ros::turtle :md5sum-) "731354329381c2f16be8e48fe990ad74")
(setf (get hello_ros::turtle :datatype-) "hello_ros/turtle")
(setf (get hello_ros::turtle :definition-)
      "string name
float32 speed
")



(provide :hello_ros/turtle "731354329381c2f16be8e48fe990ad74")


