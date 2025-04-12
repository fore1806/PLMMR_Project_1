;; Auto-generated. Do not edit!


(when (boundp 'first_project::secotor_times)
  (if (not (find-package "FIRST_PROJECT"))
    (make-package "FIRST_PROJECT"))
  (shadow 'secotor_times (find-package "FIRST_PROJECT")))
(unless (find-package "FIRST_PROJECT::SECOTOR_TIMES")
  (make-package "FIRST_PROJECT::SECOTOR_TIMES"))

(in-package "ROS")
;;//! \htmlinclude secotor_times.msg.html


(defclass first_project::secotor_times
  :super ros::object
  :slots (_current_sector _current_sector_time _current_sector_mean_speed ))

(defmethod first_project::secotor_times
  (:init
   (&key
    ((:current_sector __current_sector) 0)
    ((:current_sector_time __current_sector_time) 0.0)
    ((:current_sector_mean_speed __current_sector_mean_speed) 0.0)
    )
   (send-super :init)
   (setq _current_sector (round __current_sector))
   (setq _current_sector_time (float __current_sector_time))
   (setq _current_sector_mean_speed (float __current_sector_mean_speed))
   self)
  (:current_sector
   (&optional __current_sector)
   (if __current_sector (setq _current_sector __current_sector)) _current_sector)
  (:current_sector_time
   (&optional __current_sector_time)
   (if __current_sector_time (setq _current_sector_time __current_sector_time)) _current_sector_time)
  (:current_sector_mean_speed
   (&optional __current_sector_mean_speed)
   (if __current_sector_mean_speed (setq _current_sector_mean_speed __current_sector_mean_speed)) _current_sector_mean_speed)
  (:serialization-length
   ()
   (+
    ;; int32 _current_sector
    4
    ;; float32 _current_sector_time
    4
    ;; float32 _current_sector_mean_speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _current_sector
       (write-long _current_sector s)
     ;; float32 _current_sector_time
       (sys::poke _current_sector_time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _current_sector_mean_speed
       (sys::poke _current_sector_mean_speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _current_sector
     (setq _current_sector (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _current_sector_time
     (setq _current_sector_time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _current_sector_mean_speed
     (setq _current_sector_mean_speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get first_project::secotor_times :md5sum-) "245a0ea055f4366c2b43f7ea3395f4ec")
(setf (get first_project::secotor_times :datatype-) "first_project/secotor_times")
(setf (get first_project::secotor_times :definition-)
      "int32 current_sector
float32 current_sector_time
float32 current_sector_mean_speed

")



(provide :first_project/secotor_times "245a0ea055f4366c2b43f7ea3395f4ec")


