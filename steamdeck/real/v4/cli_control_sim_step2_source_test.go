package main

import (
	"testing"
)

func robotV4Step2TestRepositoryDirectory(t *testing.T) string {
	t.Helper()
	repositoryDirectory, err := repoRoot()
	if err != nil {
		t.Fatal(err)
	}
	return repositoryDirectory
}
