
(cl:in-package :asdf)

(defsystem "xyzpublisher-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Kenkipos" :depends-on ("_package_Kenkipos"))
    (:file "_package_Kenkipos" :depends-on ("_package"))
  ))