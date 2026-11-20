module rosieos/rt-core

go 1.24

toolchain go1.26.2

require (
	golang.org/x/sys v0.34.0
	rosieos/robotdesc v0.0.0
)

replace rosieos/robotdesc => ../robot_description/go
