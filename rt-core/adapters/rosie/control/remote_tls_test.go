//go:build linux

package control

import (
	"crypto/ecdsa"
	"crypto/elliptic"
	"crypto/rand"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"encoding/pem"
	"errors"
	"io"
	"log"
	"math/big"
	"net"
	"net/http"
	"net/url"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/internal/simfixture"
	"strings"
	"sync/atomic"
	"testing"
	"time"
)

type remoteTestPKI struct {
	files  RemoteTLSFiles
	ca     *x509.Certificate
	key    *ecdsa.PrivateKey
	serial int64
	roots  *x509.CertPool
}

func remoteTestDir(t *testing.T) string {
	t.Helper()
	root, err := filepath.Abs("../../../build")
	if err != nil {
		t.Fatal(err)
	}
	dir, err := os.MkdirTemp(root, "remote-test-")
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { os.RemoveAll(dir) })
	return dir
}

func newRemoteTestPKI(t *testing.T, pair string) *remoteTestPKI {
	t.Helper()
	dir := remoteTestDir(t)
	key, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
	if err != nil {
		t.Fatal(err)
	}
	now := time.Now()
	template := &x509.Certificate{SerialNumber: big.NewInt(1), Subject: pkix.Name{CommonName: "remote test CA"}, IsCA: true, BasicConstraintsValid: true,
		KeyUsage: x509.KeyUsageCertSign | x509.KeyUsageCRLSign, NotBefore: now.Add(-time.Hour), NotAfter: now.Add(24 * time.Hour)}
	der, err := x509.CreateCertificate(rand.Reader, template, template, &key.PublicKey, key)
	if err != nil {
		t.Fatal(err)
	}
	ca, err := x509.ParseCertificate(der)
	if err != nil {
		t.Fatal(err)
	}
	p := &remoteTestPKI{files: RemoteTLSFiles{CA: filepath.Join(dir, "ca.pem"), Certificate: filepath.Join(dir, "server.pem"), Key: filepath.Join(dir, "server-key.pem"), CRL: filepath.Join(dir, "crl.pem"), PairID: pair}, ca: ca, key: key, serial: 1, roots: x509.NewCertPool()}
	p.roots.AddCert(ca)
	p.write(t, p.files.CA, "CERTIFICATE", der)
	p.server(t)
	p.crl(t, nil)
	return p
}
func (p *remoteTestPKI) write(t *testing.T, path, kind string, der []byte) {
	t.Helper()
	if err := os.WriteFile(path, pem.EncodeToMemory(&pem.Block{Type: kind, Bytes: der}), 0600); err != nil {
		t.Fatal(err)
	}
}
func (p *remoteTestPKI) certificate(t *testing.T, principal, pair string, expired, server bool) tls.Certificate {
	t.Helper()
	p.serial++
	key, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
	if err != nil {
		t.Fatal(err)
	}
	now := time.Now()
	template := &x509.Certificate{SerialNumber: big.NewInt(p.serial), Subject: pkix.Name{CommonName: "remote test leaf"}, NotBefore: now.Add(-time.Hour), NotAfter: now.Add(time.Hour), KeyUsage: x509.KeyUsageDigitalSignature}
	if expired {
		template.NotAfter = now.Add(-time.Minute)
	}
	if server {
		template.ExtKeyUsage = []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth}
		template.IPAddresses = []net.IP{net.ParseIP("127.0.0.1")}
	} else {
		template.ExtKeyUsage = []x509.ExtKeyUsage{x509.ExtKeyUsageClientAuth}
		for scheme, value := range map[string]string{"rosie-principal": principal, "rosie-pair": pair} {
			if value != "" {
				template.URIs = append(template.URIs, &url.URL{Scheme: scheme, Opaque: value})
			}
		}
	}
	der, err := x509.CreateCertificate(rand.Reader, template, p.ca, &key.PublicKey, p.key)
	if err != nil {
		t.Fatal(err)
	}
	leaf, err := x509.ParseCertificate(der)
	if err != nil {
		t.Fatal(err)
	}
	return tls.Certificate{Certificate: [][]byte{der}, PrivateKey: key, Leaf: leaf}
}
func (p *remoteTestPKI) server(t *testing.T) tls.Certificate {
	certificate := p.certificate(t, "", "", false, true)
	p.write(t, p.files.Certificate, "CERTIFICATE", certificate.Certificate[0])
	der, err := x509.MarshalPKCS8PrivateKey(certificate.PrivateKey)
	if err != nil {
		t.Fatal(err)
	}
	p.write(t, p.files.Key, "PRIVATE KEY", der)
	return certificate
}
func (p *remoteTestPKI) crl(t *testing.T, revoked []x509.RevocationListEntry) {
	t.Helper()
	p.serial++
	der, err := x509.CreateRevocationList(rand.Reader, &x509.RevocationList{Number: big.NewInt(p.serial), ThisUpdate: time.Now().Add(-time.Minute), NextUpdate: time.Now().Add(time.Hour), RevokedCertificateEntries: revoked}, p.ca, p.key)
	if err != nil {
		t.Fatal(err)
	}
	p.write(t, p.files.CRL+".new", "X509 CRL", der)
	if err := os.Rename(p.files.CRL+".new", p.files.CRL); err != nil {
		t.Fatal(err)
	}
	// Explicit distinct mtime tests the reload contract without sleeping for filesystem resolution.
	stamp := time.Now().Add(time.Duration(p.serial) * time.Second)
	if err := os.Chtimes(p.files.CRL, stamp, stamp); err != nil {
		t.Fatal(err)
	}
}
func remoteTestServer(t *testing.T, p *remoteTestPKI, handler http.Handler) string {
	t.Helper()
	config, err := NewRemoteTLSConfig(p.files)
	if err != nil {
		t.Fatal(err)
	}
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatal(err)
	}
	server := &http.Server{Handler: handler, ErrorLog: log.New(io.Discard, "", 0)}
	done := make(chan struct{})
	go func() { defer close(done); _ = server.Serve(NewRemoteListener(listener, config)) }()
	t.Cleanup(func() { server.Close(); listener.Close(); <-done })
	return listener.Addr().String()
}
func (p *remoteTestPKI) client(t *testing.T, certificate *tls.Certificate) *http.Client {
	t.Helper()
	config := &tls.Config{RootCAs: p.roots, MinVersion: tls.VersionTLS13}
	if certificate != nil {
		config.Certificates = []tls.Certificate{*certificate}
	}
	transport := &http.Transport{TLSClientConfig: config, DisableKeepAlives: true}
	t.Cleanup(transport.CloseIdleConnections)
	return &http.Client{Transport: transport, Timeout: 3 * time.Second}
}

func TestRemoteListenerRejectsUnauthenticatedClients(t *testing.T) {
	p := newRemoteTestPKI(t, "pair")
	revoked := p.certificate(t, "revoked", "pair", false, false)
	p.crl(t, []x509.RevocationListEntry{{SerialNumber: revoked.Leaf.SerialNumber, RevocationTime: time.Now().Add(-time.Minute)}})
	other := newRemoteTestPKI(t, "pair")
	cases := map[string]*tls.Certificate{"no certificate": nil}
	for name, cert := range map[string]tls.Certificate{"another CA": other.certificate(t, "other", "pair", false, false), "expired": p.certificate(t, "expired", "pair", true, false), "revoked": revoked,
		"missing pair": p.certificate(t, "missing", "", false, false), "wrong pair": p.certificate(t, "wrong", "other-pair", false, false), "missing principal": p.certificate(t, "", "pair", false, false)} {
		c := cert
		cases[name] = &c
	}
	var stops atomic.Int64
	address := remoteTestServer(t, p, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) { stops.Add(1); _, _ = w.Write([]byte("stop reached")) }))
	for name, cert := range cases {
		t.Run(name, func(t *testing.T) {
			response, err := p.client(t, cert).Post("https://"+address+"/v1/control", "application/json", strings.NewReader(`{"schema":"rosie.rt-control.request.v1","operation":"stop"}`))
			if response != nil {
				body, _ := io.ReadAll(response.Body)
				response.Body.Close()
				if len(body) != 0 {
					t.Fatalf("unauthenticated stop response: %s (%v)", body, err)
				}
			}
			if stops.Load() != 0 {
				t.Fatal("unauthenticated stop reached HTTP handler")
			}
		})
	}
	t.Run("plaintext stop", func(t *testing.T) {
		conn, err := net.Dial("tcp", address)
		if err != nil {
			t.Fatal(err)
		}
		defer conn.Close()
		_ = conn.SetDeadline(time.Now().Add(time.Second))
		_, err = io.WriteString(conn, "POST /v1/control HTTP/1.1\r\nHost: localhost\r\nContent-Length: 20\r\n\r\n{\"operation\":\"stop\"}")
		if err != nil {
			t.Fatal(err)
		}
		body, readErr := io.ReadAll(conn)
		if readErr != nil || len(body) != 0 || stops.Load() != 0 {
			t.Fatalf("unauthenticated plaintext response: %q %v", body, readErr)
		}
	})
}

func TestRemotePkiRotationReloadsCertificates(t *testing.T) {
	p := newRemoteTestPKI(t, "pair")
	cert := p.certificate(t, "pendant", "pair", false, false)
	address := remoteTestServer(t, p, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) { _, _ = w.Write([]byte("authenticated")) }))
	client := p.client(t, &cert)
	response, err := client.Get("https://" + address)
	if err != nil {
		t.Fatal(err)
	}
	response.Body.Close()
	original := response.TLS.PeerCertificates[0].SerialNumber.String()
	replacement := p.server(t)
	stamp := time.Now().Add(time.Minute)
	if err := os.Chtimes(p.files.Certificate, stamp, stamp); err != nil {
		t.Fatal(err)
	}
	response, err = client.Get("https://" + address)
	if err != nil {
		t.Fatal(err)
	}
	response.Body.Close()
	if response.TLS.PeerCertificates[0].SerialNumber.String() == original || response.TLS.PeerCertificates[0].SerialNumber.Cmp(replacement.Leaf.SerialNumber) != 0 {
		t.Fatal("server certificate was not reloaded")
	}
	p.crl(t, []x509.RevocationListEntry{{SerialNumber: cert.Leaf.SerialNumber, RevocationTime: time.Now()}})
	response, err = client.Get("https://" + address)
	if response != nil {
		response.Body.Close()
	}
	if err == nil {
		t.Fatal("revoked client accepted after CRL rotation")
	}
}

func TestRemoteTLSMissingFilesFailClosed(t *testing.T) {
	p := newRemoteTestPKI(t, "pair")
	for _, field := range []string{"CA", "Certificate", "Key", "CRL"} {
		t.Run(field, func(t *testing.T) {
			files := p.files
			switch field {
			case "CA":
				files.CA = ""
			case "Certificate":
				files.Certificate = ""
			case "Key":
				files.Key = ""
			case "CRL":
				files.CRL = ""
			}
			if _, err := NewRemoteTLSConfig(files); err == nil {
				t.Fatal("missing credential accepted")
			}
		})
	}
	if err := os.WriteFile(p.files.CRL, []byte("corrupt"), 0600); err != nil {
		t.Fatal(err)
	}
	if _, err := NewRemoteTLSConfig(p.files); err == nil {
		t.Fatal("corrupt CRL accepted")
	}
}

func TestRemoteIdentityRejectsAmbiguousSANs(t *testing.T) {
	for _, invalid := range []string{"rosie-principal:pendant?ignored", "rosie-principal:pendant#ignored", "rosie-principal://pendant"} {
		t.Run(invalid, func(t *testing.T) {
			uri, err := url.Parse(invalid)
			if err != nil {
				t.Fatal(err)
			}
			certificate := &x509.Certificate{URIs: []*url.URL{uri, {Scheme: "rosie-pair", Opaque: "pair"}}}
			if _, err := remoteIdentity(certificate, "pair"); err == nil {
				t.Fatal("ambiguous SAN accepted")
			}
		})
	}
}

func TestRemoteStartupMissingCredentialsExitTwo(t *testing.T) {
	p := newRemoteTestPKI(t, "pair")
	binary, err := filepath.Abs(simfixture.Locate("../../..", "rt-control"))
	if err != nil {
		t.Fatal(err)
	}
	for _, missing := range []string{"--remote-ca", "--remote-cert", "--remote-key", "--remote-crl"} {
		t.Run(missing, func(t *testing.T) {
			args := []string{"--remote-listen", "127.0.0.1:0", "--pair-id", "pair", "--core-socket", filepath.Join(remoteTestDir(t), "absent.sock")}
			for flag, path := range map[string]string{"--remote-ca": p.files.CA, "--remote-cert": p.files.Certificate, "--remote-key": p.files.Key, "--remote-crl": p.files.CRL} {
				if flag != missing {
					args = append(args, flag, path)
				}
			}
			output, err := exec.Command(binary, args...).CombinedOutput()
			var exit *exec.ExitError
			if !errors.As(err, &exit) || exit.ExitCode() != 2 {
				t.Fatalf("startup must exit 2 before native dial: %s %v", output, err)
			}
		})
	}
}
