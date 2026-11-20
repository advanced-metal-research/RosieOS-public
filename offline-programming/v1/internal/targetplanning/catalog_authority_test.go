package targetplanning

import (
	"context"
	"encoding/json"
	"testing"

	"rosieos/offline-programming/v1/internal/catalog"
)

type fixtureCatalogGateway struct {
	response catalog.Response
	program  string
	revision uint64
}

func (*fixtureCatalogGateway) Available() bool { return true }

func (*fixtureCatalogGateway) List(context.Context) (catalog.Response, error) {
	return catalog.Response{}, nil
}

func (gateway *fixtureCatalogGateway) Get(
	_ context.Context,
	program string,
	revision *uint64,
) (catalog.Response, error) {
	gateway.program = program
	if revision != nil {
		gateway.revision = *revision
	}
	return gateway.response, nil
}

func (*fixtureCatalogGateway) Revisions(
	context.Context,
	string,
) (catalog.Response, error) {
	return catalog.Response{}, nil
}

func (*fixtureCatalogGateway) Save(
	context.Context,
	[]byte,
	catalog.MutationContext,
) (catalog.Response, error) {
	return catalog.Response{}, nil
}

func TestCatalogProgramAuthorityReturnsExactSavedRevision(t *testing.T) {
	saved := json.RawMessage(
		"{\n  \"program_id\":\"program-1\",\"program_revision\":4\n}",
	)
	entry := append(
		[]byte(`{"schema":"robot.v4.program-entry.v1","owner":"motion-server","program":`),
		saved...,
	)
	entry = append(entry, '}')
	gateway := &fixtureCatalogGateway{
		response: catalog.Response{Status: 200, Body: entry},
	}
	authority, err := NewCatalogProgramAuthority(gateway)
	if err != nil {
		t.Fatal(err)
	}
	program, err := authority.SavedProgram(
		context.Background(),
		"program-1",
		4,
	)
	if err != nil {
		t.Fatal(err)
	}
	if string(program) != string(saved) ||
		gateway.program != "program-1" ||
		gateway.revision != 4 {
		t.Fatalf(
			"program=%q identity=%q r%d",
			program,
			gateway.program,
			gateway.revision,
		)
	}
}
