;; Auto-generated. Do not edit!


(when (boundp 'xyzpublisher::Kenkipos)
  (if (not (find-package "XYZPUBLISHER"))
    (make-package "XYZPUBLISHER"))
  (shadow 'Kenkipos (find-package "XYZPUBLISHER")))
(unless (find-package "XYZPUBLISHER::KENKIPOS")
  (make-package "XYZPUBLISHER::KENKIPOS"))

(in-package "ROS")
;;//! \htmlinclude Kenkipos.msg.html


(defclass xyzpublisher::Kenkipos
  :super ros::object
  :slots (_stamp _x _y _z ))

(defmethod xyzpublisher::Kenkipos
  (:init
   (&key
    ((:stamp __stamp) (instance ros::time :init))
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; time _stamp
    8
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64 _z
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z
       (sys::poke _z (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z
     (setq _z (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get xyzpublisher::Kenkipos :md5sum-) "feb38048784c133ece3b9bddb7dd3f47")
(setf (get xyzpublisher::Kenkipos :datatype-) "xyzpublisher/Kenkipos")
(setf (get xyzpublisher::Kenkipos :definition-)
      "#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
float64 x
float64 y
float64 z

")



(provide :xyzpublisher/Kenkipos "feb38048784c133ece3b9bddb7dd3f47")


