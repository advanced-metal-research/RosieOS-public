//go:build linux

package ipcclient

// legacyIdentityArgs keeps the existing native parser/fault-injection oracle on
// its optional-digest invocation when it intentionally edits compiled inputs.
// The daemon still computes its effective machine digest; identity refusal tests
// use independently compiled expected digests instead of this helper.
func legacyIdentityArgs(args []string) []string {
	result := make([]string, 0, len(args))
	for i := 0; i < len(args); i++ {
		switch args[i] {
		case "--machine-sha256", "--machine-identity", "--deployment-sha256":
			i++
			continue
		}
		result = append(result, args[i])
	}
	return result
}
