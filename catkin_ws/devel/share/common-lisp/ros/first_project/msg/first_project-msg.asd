
(cl:in-package :asdf)

(defsystem "first_project-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "secotor_times" :depends-on ("_package_secotor_times"))
    (:file "_package_secotor_times" :depends-on ("_package"))
  ))