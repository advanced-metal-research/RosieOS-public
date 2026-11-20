//go:build linux

package control

import (
	"crypto/tls"
	"crypto/x509"
	"encoding/pem"
	"errors"
	"fmt"
	"os"
	"regexp"
	"sync"
	"time"
)

type RemoteTLSFiles struct{ CA, Certificate, Key, CRL, PairID string }

var remoteToken = regexp.MustCompile(`^[A-Za-z0-9_.:-]+$`)

// remoteIdentity requires unique component-owned URI bindings; ambiguous SANs
// cannot confer either a principal or a deployment authority.
func remoteIdentity(cert *x509.Certificate, pair string) (string, error) {
	principal, binding := "", ""
	for _, uri := range cert.URIs {
		if (uri.Scheme == "rosie-principal" || uri.Scheme == "rosie-pair") &&
			(uri.Host != "" || uri.User != nil || uri.Path != "" || uri.RawQuery != "" || uri.ForceQuery || uri.Fragment != "") {
			return "", errors.New("remote identity SAN must be an opaque token")
		}
		switch uri.Scheme {
		case "rosie-principal":
			if principal != "" || !remoteToken.MatchString(uri.Opaque) {
				return "", errors.New("invalid remote principal SAN")
			}
			principal = uri.Opaque
		case "rosie-pair":
			if binding != "" || !remoteToken.MatchString(uri.Opaque) {
				return "", errors.New("invalid remote pair SAN")
			}
			binding = uri.Opaque
		}
	}
	if principal == "" || binding == "" || binding != pair {
		return "", errors.New("remote principal/pair binding mismatch")
	}
	return principal, nil
}

// NewRemoteTLSConfig validates every configured file before listening and reloads
// changed certificate/key/CRL files before each full handshake. Failed reloads
// never fall back to stale credentials or a previously valid revocation list.
func NewRemoteTLSConfig(files RemoteTLSFiles) (*tls.Config, error) {
	if !remoteToken.MatchString(files.PairID) {
		return nil, errors.New("remote pair-id required")
	}
	paths := []string{files.CA, files.Certificate, files.Key, files.CRL}
	for _, path := range paths {
		if path == "" {
			return nil, errors.New("remote CA, certificate, key and CRL are required")
		}
	}
	caPEM, err := os.ReadFile(files.CA)
	if err != nil {
		return nil, err
	}
	block, rest := pem.Decode(caPEM)
	if block == nil || block.Type != "CERTIFICATE" || len(rest) != 0 {
		return nil, errors.New("remote CA must contain one CA certificate")
	}
	ca, err := x509.ParseCertificate(block.Bytes)
	if err != nil {
		return nil, err
	}
	if !ca.IsCA || ca.CheckSignatureFrom(ca) != nil {
		return nil, errors.New("remote CA must be a self-signed component CA")
	}
	roots := x509.NewCertPool()
	roots.AddCert(ca)
	var mu sync.Mutex
	var stamps [3]time.Time
	var cached *tls.Config
	load := func() (*tls.Config, error) {
		mu.Lock()
		defer mu.Unlock()
		var next [3]time.Time
		for i, path := range paths[1:] {
			info, err := os.Stat(path)
			if err != nil {
				return nil, err
			}
			next[i] = info.ModTime()
		}
		if cached != nil && next == stamps {
			return cached, nil
		}
		certificate, err := tls.LoadX509KeyPair(files.Certificate, files.Key)
		if err != nil {
			return nil, err
		}
		raw, err := os.ReadFile(files.CRL)
		if err != nil {
			return nil, err
		}
		block, _ := pem.Decode(raw)
		if block == nil || block.Type != "X509 CRL" {
			return nil, errors.New("invalid remote CRL PEM")
		}
		crl, err := x509.ParseRevocationList(block.Bytes)
		if err != nil {
			return nil, err
		}
		if err = crl.CheckSignatureFrom(ca); err != nil {
			return nil, fmt.Errorf("remote CRL signature: %w", err)
		}
		for _, extension := range crl.Extensions {
			if extension.Critical {
				return nil, errors.New("unsupported critical CRL extension")
			}
		}
		checkCRL := func() error {
			now := time.Now()
			if now.Before(crl.ThisUpdate) || !now.Before(crl.NextUpdate) {
				return errors.New("remote CRL not current")
			}
			return nil
		}
		if err = checkCRL(); err != nil {
			return nil, err
		}
		config := &tls.Config{MinVersion: tls.VersionTLS13, ClientAuth: tls.RequireAndVerifyClientCert,
			ClientCAs: roots, Certificates: []tls.Certificate{certificate}, SessionTicketsDisabled: true, NextProtos: []string{"http/1.1"}}
		config.VerifyPeerCertificate = func(_ [][]byte, chains [][]*x509.Certificate) error {
			if err := checkCRL(); err != nil {
				return err
			}
			if len(chains) == 0 || len(chains[0]) != 2 {
				return errors.New("remote client must be issued directly by component CA")
			}
			leaf := chains[0][0]
			if _, err := remoteIdentity(leaf, files.PairID); err != nil {
				return err
			}
			for _, entry := range crl.RevokedCertificateEntries {
				if entry.SerialNumber.Cmp(leaf.SerialNumber) == 0 {
					return errors.New("remote client revoked")
				}
			}
			return nil
		}
		cached, stamps = config, next
		return config, nil
	}
	initial, err := load()
	if err != nil {
		return nil, err
	}
	config := initial.Clone()
	config.GetConfigForClient = func(*tls.ClientHelloInfo) (*tls.Config, error) { return load() }
	return config, nil
}
