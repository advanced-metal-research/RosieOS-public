package main

type robotV4DeployLoggedError struct {
	err error
}

func (err *robotV4DeployLoggedError) Error() string {
	return err.err.Error()
}

func (err *robotV4DeployLoggedError) Unwrap() error {
	return err.err
}
