
(cl:in-package :asdf)

(defsystem "airhockey_main-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ArmAngles" :depends-on ("_package_ArmAngles"))
    (:file "_package_ArmAngles" :depends-on ("_package"))
  ))