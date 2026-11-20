module rosieos/offline-programming/v1

go 1.24

toolchain go1.26.2

require (
	github.com/nats-io/nats.go v1.48.0
	golang.org/x/sys v0.34.0
	rosieos/rt-core v0.0.0
	rosieos/robotdesc v0.0.0
)

require (
	github.com/klauspost/compress v1.18.0 // indirect
	github.com/nats-io/nkeys v0.4.11 // indirect
	github.com/nats-io/nuid v1.0.1 // indirect
	golang.org/x/crypto v0.37.0 // indirect
)

replace rosieos/rt-core => ../../rt-core

replace rosieos/robotdesc => ../../robot_description/go
