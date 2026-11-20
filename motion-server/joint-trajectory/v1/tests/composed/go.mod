module rosieos/joint-trajectory-v1-tests

go 1.26.2

require (
	rosieos/rt-core v0.0.0
	rosieos/robotdesc v0.0.0
)

require golang.org/x/sys v0.34.0

replace rosieos/rt-core => ../../../../../rt-core

replace rosieos/robotdesc => ../../../../../robot_description/go
