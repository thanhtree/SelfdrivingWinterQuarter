;; Auto-generated. Do not edit!


(when (boundp 'data_collection::Service)
  (if (not (find-package "DATA_COLLECTION"))
    (make-package "DATA_COLLECTION"))
  (shadow 'Service (find-package "DATA_COLLECTION")))
(unless (find-package "DATA_COLLECTION::SERVICE")
  (make-package "DATA_COLLECTION::SERVICE"))
(unless (find-package "DATA_COLLECTION::SERVICEREQUEST")
  (make-package "DATA_COLLECTION::SERVICEREQUEST"))
(unless (find-package "DATA_COLLECTION::SERVICERESPONSE")
  (make-package "DATA_COLLECTION::SERVICERESPONSE"))

(in-package "ROS")





(defclass data_collection::ServiceRequest
  :super ros::object
  :slots (_num ))

(defmethod data_collection::ServiceRequest
  (:init
   (&key
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _num (round __num))
   self)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; int64 _num
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _num
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num) (= (length (_num . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num . bv) 0) 0) s)
              (write-long (ash (elt (_num . bv) 1) -1) s))
             ((and (class _num) (= (length (_num . bv)) 1)) ;; big1
              (write-long (elt (_num . bv) 0) s)
              (write-long (if (>= _num 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num s)(write-long (if (>= _num 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _num
#+(or :alpha :irix6 :x86_64)
      (setf _num (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass data_collection::ServiceResponse
  :super ros::object
  :slots ())

(defmethod data_collection::ServiceResponse
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

(defclass data_collection::Service
  :super ros::object
  :slots ())

(setf (get data_collection::Service :md5sum-) "57d3c40ec3ac3754af76a83e6e73127a")
(setf (get data_collection::Service :datatype-) "data_collection/Service")
(setf (get data_collection::Service :request) data_collection::ServiceRequest)
(setf (get data_collection::Service :response) data_collection::ServiceResponse)

(defmethod data_collection::ServiceRequest
  (:response () (instance data_collection::ServiceResponse :init)))

(setf (get data_collection::ServiceRequest :md5sum-) "57d3c40ec3ac3754af76a83e6e73127a")
(setf (get data_collection::ServiceRequest :datatype-) "data_collection/ServiceRequest")
(setf (get data_collection::ServiceRequest :definition-)
      "int64 num
---
")

(setf (get data_collection::ServiceResponse :md5sum-) "57d3c40ec3ac3754af76a83e6e73127a")
(setf (get data_collection::ServiceResponse :datatype-) "data_collection/ServiceResponse")
(setf (get data_collection::ServiceResponse :definition-)
      "int64 num
---
")



(provide :data_collection/Service "57d3c40ec3ac3754af76a83e6e73127a")


