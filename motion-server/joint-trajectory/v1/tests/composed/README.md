Composed simulation requires at least two allowed CPUs: one RT CPU and one
housekeeping CPU. With fewer allowed CPUs, the suite reports an explicit
`CPURequirementError` skip before building or starting daemons. The daemon's
housekeeping admission remains mandatory.
