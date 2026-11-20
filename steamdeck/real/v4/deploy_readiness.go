package main

import (
	"strings"
)

func robotV4DeployObservedIdentityMatchesPeer(observed, targetPeer string) bool {
	observed = strings.ToLower(strings.TrimSpace(observed))
	targetPeer = robotV4DeployCanonicalPeerName(targetPeer)
	if observed == "" || targetPeer == "" {
		return false
	}
	if observed == targetPeer {
		return true
	}
	node, ok := robotV4DaemonNodeInventoryRecordByAlias(targetPeer)
	if !ok {
		return false
	}
	for _, concrete := range robotV4DaemonConcreteIdentityAliases(node) {
		if strings.EqualFold(strings.TrimSpace(concrete), observed) {
			return true
		}
	}
	return false
}
