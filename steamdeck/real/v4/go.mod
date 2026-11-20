module rosieos/steamdeck/real/v4

go 1.26.2

require golang.org/x/sys v0.34.0

require (
	rosieos/rt-core v0.0.0
	rosieos/robotdesc v0.0.0
)

replace rosieos/rt-core => ../../../rt-core

replace rosieos/robotdesc => ../../../robot_description/go
