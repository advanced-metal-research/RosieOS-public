//go:build linux

package control

import (
	"context"
	"crypto/ecdsa"
	"crypto/elliptic"
	"crypto/rand"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"encoding/pem"
	"errors"
	"math/big"
	"net"
	"net/url"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"
)

type testPKI struct {
	dir    string
	roots  *x509.CertPool
	client tls.Certificate
}

func newPKI(t *testing.T) testPKI {
	t.Helper()
	root, err := filepath.Abs("../../build")
	must(t, err)
	dir, err := os.MkdirTemp(root, "sdk-pki-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	key, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
	must(t, err)
	now := time.Now()
	ca := &x509.Certificate{SerialNumber: big.NewInt(1), Subject: pkix.Name{CommonName: "sdk test CA"}, IsCA: true, BasicConstraintsValid: true, KeyUsage: x509.KeyUsageCertSign | x509.KeyUsageCRLSign, NotBefore: now.Add(-time.Hour), NotAfter: now.Add(time.Hour)}
	der, err := x509.CreateCertificate(rand.Reader, ca, ca, &key.PublicKey, key)
	must(t, err)
	ca, err = x509.ParseCertificate(der)
	must(t, err)
	write := func(name, kind string, raw []byte) {
		must(t, os.WriteFile(filepath.Join(dir, name), pem.EncodeToMemory(&pem.Block{Type: kind, Bytes: raw}), 0600))
	}
	write("ca.pem", "CERTIFICATE", der)
	roots := x509.NewCertPool()
	roots.AddCert(ca)
	makeCert := func(name string, serial int64, server bool) tls.Certificate {
		leafKey, err := ecdsa.GenerateKey(elliptic.P256(), rand.Reader)
		must(t, err)
		leaf := &x509.Certificate{SerialNumber: big.NewInt(serial), NotBefore: now.Add(-time.Hour), NotAfter: now.Add(time.Hour), KeyUsage: x509.KeyUsageDigitalSignature}
		if server {
			leaf.ExtKeyUsage = []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth}
			leaf.IPAddresses = []net.IP{net.ParseIP("127.0.0.1")}
		} else {
			leaf.ExtKeyUsage = []x509.ExtKeyUsage{x509.ExtKeyUsageClientAuth}
			leaf.URIs = []*url.URL{{Scheme: "rosie-principal", Opaque: "sdk"}, {Scheme: "rosie-pair", Opaque: "bench"}}
		}
		der, err := x509.CreateCertificate(rand.Reader, leaf, ca, &leafKey.PublicKey, key)
		must(t, err)
		write(name+".pem", "CERTIFICATE", der)
		private, err := x509.MarshalPKCS8PrivateKey(leafKey)
		must(t, err)
		write(name+"-key.pem", "PRIVATE KEY", private)
		return tls.Certificate{Certificate: [][]byte{der}, PrivateKey: leafKey}
	}
	makeCert("server", 2, true)
	client := makeCert("client", 3, false)
	crl, err := x509.CreateRevocationList(rand.Reader, &x509.RevocationList{Number: big.NewInt(1), ThisUpdate: now.Add(-time.Minute), NextUpdate: now.Add(time.Hour)}, ca, key)
	must(t, err)
	write("crl.pem", "X509 CRL", crl)
	return testPKI{dir: dir, roots: roots, client: client}
}

func (p testPKI) remoteArgs(t *testing.T) (string, []string) {
	t.Helper()
	listener, err := net.Listen("tcp", "127.0.0.1:0")
	must(t, err)
	addr := listener.Addr().String()
	must(t, listener.Close())
	return addr, []string{"--remote-listen", addr, "--remote-ca", filepath.Join(p.dir, "ca.pem"), "--remote-cert", filepath.Join(p.dir, "server.pem"), "--remote-key", filepath.Join(p.dir, "server-key.pem"), "--remote-crl", filepath.Join(p.dir, "crl.pem")}
}

func TestSDKRemoteDialRequiresClientCertificate(t *testing.T) {
	p := newPKI(t)
	addr, args := p.remoteArgs(t)
	s := startService(t, args...)
	_, err := DialTLS(addr, &tls.Config{RootCAs: p.roots})
	if !errors.Is(err, ErrTransport) {
		t.Fatalf("accepted missing client certificate: %v", err)
	}
	// An empty dynamic certificate exercises the real listener's handshake,
	// independently of the constructor's missing-credential check.
	empty, err := DialTLS(addr, &tls.Config{RootCAs: p.roots, GetClientCertificate: func(*tls.CertificateRequestInfo) (*tls.Certificate, error) { return &tls.Certificate{}, nil }})
	must(t, err)
	defer empty.Close()
	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()
	_, err = empty.Describe(ctx)
	// TLS may reject before HTTP bytes are written. Both transport categories
	// must retain the certificate refusal, never merely fail for another reason.
	if (!errors.Is(err, ErrTransport) && !errors.Is(err, ErrConnectionUnsent)) || !strings.Contains(err.Error(), "tls: certificate required") {
		t.Fatalf("listener accepted empty certificate: %v", err)
	}
	client, err := DialTLS(addr, &tls.Config{RootCAs: p.roots, Certificates: []tls.Certificate{p.client}})
	must(t, err)
	defer client.Close()
	description, err := client.Describe(ctx)
	must(t, err)
	if description.ConfigurationSHA256 != s.hash {
		t.Fatal("remote description identity")
	}
	_, err = client.Acquire(ctx, "sdk-remote", Binding{PairID: "bench", Revision: 7, ConfigurationSHA256: s.hash})
	must(t, err)
	_, err = client.StartRenewal(ctx, 50*time.Millisecond)
	must(t, err)
	_, err = client.PrepareProgram(ctx, oracleProgram(t, s.root, false))
	must(t, err)
	_, err = client.Status(ctx)
	must(t, err)
	_, err = client.Stop(ctx)
	must(t, err)
	_, err = client.Release(ctx)
	must(t, err)
	runRtctl(t, s, 0, "describe", "--remote", addr, "--cert", filepath.Join(p.dir, "client.pem"), "--key", filepath.Join(p.dir, "client-key.pem"), "--ca", filepath.Join(p.dir, "ca.pem"), "--json")
}
