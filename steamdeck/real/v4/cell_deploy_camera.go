package main

import (
	"encoding/base64"
	"flag"
	"fmt"
	"io"
	"net"
	"regexp"
	"strconv"
	"strings"
)

// The cell camera (cell-camera/v1) on the cell's relay: `camera install` and
// `camera status`. Every value comes from the manifest's cell_deploy.camera
// block, which is also where the OLP unit's camera origin comes from, so the
// service and the OLP that relays it cannot be told two different things.
// Touches no controller: a camera is a view of the cell, with no robot
// interface.

// cellDeployCamera is one camera on the relay. Field meanings are the
// service's own settings (cell-camera/v1/README.md); the service re-checks
// every value before it starts.
type cellDeployCamera struct {
	// Instance names the systemd unit: rosie-cell-camera@<instance>.service.
	Instance string `json:"instance"`
	// Device is a V4L2 camera's node; absent for a RealSense.
	Device string `json:"device,omitempty"`
	// Source is "v4l2" (the default: Device) or "realsense": an Intel
	// RealSense's colour camera through librealsense, chosen by RealSense's
	// serial number (cell-camera/v1/README.md, "RealSense").
	Source      string                     `json:"source,omitempty"`
	RealSense   *cellDeployCameraRealSense `json:"realsense,omitempty"`
	Capture     string                     `json:"capture"`
	PixelFormat string                     `json:"pixel_format,omitempty"`
	Output      string                     `json:"output,omitempty"`
	JPEGQuality int                        `json:"jpeg_quality,omitempty"`
	Encoder     string                     `json:"encoder,omitempty"`
	Listen      string                     `json:"listen"`
	PathPrefix  string                     `json:"path_prefix"`
	Label       string                     `json:"label,omitempty"`
	Title       string                     `json:"title,omitempty"`
	// Replaces are units that hold the same camera or port and are stopped and
	// disabled before this one starts (an older service not from this
	// repository). A failed install starts them again.
	Replaces []string `json:"replaces,omitempty"`
	// Video turns on the low-data H.264 stream and its WebRTC sender,
	// rosie-cell-video@<instance> (cell-camera/v1/video). Absent: JPEG only.
	Video *cellDeployCameraVideo `json:"video,omitempty"`
}

// cellDeployCameraRealSense is a RealSense camera's own block. Serial picks
// the camera, or empty: the only RealSense on the relay. SDKDir is the
// pyrealsense2 package cell-camera/v1/host/build-pyrealsense2.sh built on the
// relay; camera install copies it beside the service. Depth is the depth
// stream's WIDTHxHEIGHT@FPS, read in the colour's librealsense session and
// served raw at <path_prefix>/depth.z16, or empty: colour only.
type cellDeployCameraRealSense struct {
	Serial string `json:"serial,omitempty"`
	SDKDir string `json:"sdk_dir"`
	Depth  string `json:"depth,omitempty"`
}

// Camera is which camera the block reads: the V4L2 node, or the RealSense by
// serial number. Two camera blocks of a cell never read the same one.
func (c cellDeployCamera) Camera() string {
	if c.Source != "realsense" {
		return c.Device
	}
	if c.RealSense == nil || c.RealSense.Serial == "" {
		return "the only RealSense"
	}
	return "RealSense " + c.RealSense.Serial
}

// cellDeployCameraVideo is the camera's low-data video (cell-camera/v1
// README "Low-data video"). Address "tailscale" is the relay's own
// Tailscale IPv4 address, read on the relay at install.
type cellDeployCameraVideo struct {
	Address string `json:"address"`
	Port    int    `json:"port,omitempty"`
	Listen  string `json:"listen"`
	Width   int    `json:"width,omitempty"`
	FPS     int    `json:"fps,omitempty"`
	KBPS    int    `json:"kbps,omitempty"`
}

// cellDeployTailscaleAddress stands for the relay's Tailscale IPv4 address
// in a rendered settings file until the install script reads it.
const cellDeployTailscaleAddress = "@TAILSCALE_IPV4@"

func (v cellDeployCameraVideo) validate() error {
	bad := func(field, want, got string) error {
		return fmt.Errorf("cell_deploy camera.video.%s must be %s, got %q", field, want, got)
	}
	if ip := net.ParseIP(v.Address); v.Address != "tailscale" && (ip == nil || ip.IsLoopback() || ip.IsUnspecified()) {
		return bad("address", `"tailscale" or the relay's Tailscale IP`, v.Address)
	}
	if v.Port != 0 && (v.Port < 1024 || v.Port > 65535) {
		return bad("port", "1024..65535", strconv.Itoa(v.Port))
	}
	host, port, err := net.SplitHostPort(v.Listen)
	n, _ := strconv.Atoi(port)
	if ip := net.ParseIP(host); err != nil || ip == nil || !ip.IsLoopback() || n < 1024 {
		return bad("listen", "a loopback host:port (the OLP relays it)", v.Listen)
	}
	for field, value := range map[string][3]int{"width": {v.Width, 160, 7680}, "fps": {v.FPS, 1, 120}, "kbps": {v.KBPS, 50, 20000}} {
		if value[0] != 0 && (value[0] < value[1] || value[0] > value[2]) {
			return bad(field, fmt.Sprintf("%d..%d", value[1], value[2]), strconv.Itoa(value[0]))
		}
	}
	return nil
}

var (
	cellDeployCameraInstance = regexp.MustCompile(`^[a-z0-9][a-z0-9-]{0,31}$`)
	cellDeployCameraDevice   = regexp.MustCompile(`^/dev/[A-Za-z0-9._:+,/-]+$`)
	cellDeployCameraMode     = regexp.MustCompile(`^\d{2,5}x\d{2,5}@\d{1,3}$`)
	cellDeployCameraSize     = regexp.MustCompile(`^\d{2,5}x\d{2,5}$`)
	cellDeployCameraFormat   = regexp.MustCompile(`^[A-Z0-9]{3,8}$`)
	cellDeployCameraPath     = regexp.MustCompile(`^(/[A-Za-z0-9._-]+)+$`)
	cellDeployCameraText     = regexp.MustCompile(`^[^\x00-\x1f<>"&'\\=]{1,80}$`)
	cellDeployCameraUnit     = regexp.MustCompile(`^[A-Za-z0-9@._-]+\.service$`)
	cellDeployCameraSerial   = regexp.MustCompile(`^[A-Za-z0-9]{1,32}$`)
	cellDeployCameraDir      = regexp.MustCompile(`^(/[A-Za-z0-9._+-]+)+$`)
)

func (c cellDeployCamera) validate() error {
	bad := func(field, want, got string) error {
		return fmt.Errorf("cell_deploy camera.%s must be %s, got %q", field, want, got)
	}
	if !cellDeployCameraInstance.MatchString(c.Instance) {
		return bad("instance", "lower-case letters, digits and dashes", c.Instance)
	}
	switch c.Source {
	case "", "v4l2":
		if c.RealSense != nil {
			return bad("realsense", `absent unless source is "realsense"`, "a block")
		}
		if !cellDeployCameraDevice.MatchString(c.Device) || strings.Contains(c.Device, "..") {
			return bad("device", "a device path under /dev (use /dev/v4l/by-id/...)", c.Device)
		}
	case "realsense":
		if c.Device != "" {
			return bad("device", "absent for a RealSense (realsense.serial picks it)", c.Device)
		}
		if c.RealSense == nil {
			return bad("realsense", `a block with sdk_dir for source "realsense"`, "none")
		}
		if c.RealSense.Serial != "" && !cellDeployCameraSerial.MatchString(c.RealSense.Serial) {
			return bad("realsense.serial", "the camera's serial number (letters and digits)", c.RealSense.Serial)
		}
		if !cellDeployCameraDir.MatchString(c.RealSense.SDKDir) || strings.Contains(c.RealSense.SDKDir, "..") {
			return bad("realsense.sdk_dir", "an absolute directory on the relay (cell-camera/v1/host/build-pyrealsense2.sh's output)", c.RealSense.SDKDir)
		}
		if c.RealSense.Depth != "" && !cellDeployCameraMode.MatchString(c.RealSense.Depth) {
			return bad("realsense.depth", "WIDTHxHEIGHT@FPS, for example 640x480@15, or absent for colour only", c.RealSense.Depth)
		}
		if c.PixelFormat != "" && c.PixelFormat != "YUY2" {
			return bad("pixel_format", "YUY2 for a RealSense (its colour camera's native format)", c.PixelFormat)
		}
	default:
		return bad("source", "v4l2 or realsense", c.Source)
	}
	if !cellDeployCameraMode.MatchString(c.Capture) {
		return bad("capture", "WIDTHxHEIGHT@FPS, for example 1920x1200@30", c.Capture)
	}
	if c.PixelFormat != "" && !cellDeployCameraFormat.MatchString(c.PixelFormat) {
		return bad("pixel_format", "a GStreamer raw format such as YUY2", c.PixelFormat)
	}
	if c.Output != "" && !cellDeployCameraSize.MatchString(c.Output) {
		return bad("output", "WIDTHxHEIGHT", c.Output)
	}
	if c.JPEGQuality < 0 || c.JPEGQuality > 100 {
		return bad("jpeg_quality", "1..100", strconv.Itoa(c.JPEGQuality))
	}
	if c.Encoder != "" && c.Encoder != "auto" && c.Encoder != "nvidia" && c.Encoder != "software" {
		return bad("encoder", "auto, nvidia or software", c.Encoder)
	}
	host, port, err := net.SplitHostPort(c.Listen)
	n, _ := strconv.Atoi(port)
	if ip := net.ParseIP(host); err != nil || n < 1024 || n > 65535 || (host != "localhost" && (ip == nil || !ip.IsLoopback())) {
		return bad("listen", "a loopback address and a port 1024..65535 (the OLP on the relay is its only client)", c.Listen)
	}
	if !cellDeployCameraPath.MatchString(c.PathPrefix) {
		return bad("path_prefix", "a path such as /camera", c.PathPrefix)
	}
	for field, value := range map[string]string{"label": c.Label, "title": c.Title} {
		if value != "" && !cellDeployCameraText.MatchString(value) {
			return bad(field, `1..80 printable characters without < > " & ' \ =`, value)
		}
	}
	for _, unit := range c.Replaces {
		if !cellDeployCameraUnit.MatchString(unit) || unit == c.Unit() {
			return bad("replaces", "other systemd service unit names", unit)
		}
	}
	if c.Video != nil {
		if err := c.Video.validate(); err != nil {
			return err
		}
		if c.Video.Listen == c.Listen {
			return bad("video.listen", "a different port from the camera's listen", c.Video.Listen)
		}
	}
	return nil
}

// VideoUnit is the camera's WebRTC sender unit, "" without video.
func (c cellDeployCamera) VideoUnit() string {
	if c.Video == nil {
		return ""
	}
	return "rosie-cell-video@" + c.Instance + ".service"
}

// VideoOrigin is where the OLP relays the camera's WebRTC set-up, "" without video.
func (c cellDeployCamera) VideoOrigin() string {
	if c.Video == nil {
		return ""
	}
	return "http://" + c.Video.Listen
}

// Unit is the camera's systemd unit on the relay.
func (c cellDeployCamera) Unit() string { return "rosie-cell-camera@" + c.Instance + ".service" }

// Origin is where the OLP on the relay reads the camera.
func (c cellDeployCamera) Origin() string { return "http://" + c.Listen + c.PathPrefix }

// Settings renders /etc/rosie-cell-camera/<instance>.env: one KEY=value per
// line, unquoted, as the service and systemd's EnvironmentFile= read it.
func (c cellDeployCamera) Settings() string {
	lines := []string{"# Rendered from the cell manifest's camera block for instance " + c.Instance + " by `camera install`; edit the manifest, not this file."}
	add := func(key, value string) {
		if value != "" {
			lines = append(lines, "CELL_CAMERA_"+key+"="+value)
		}
	}
	add("DEVICE", c.Device)
	add("SOURCE", c.Source)
	if c.RealSense != nil {
		add("REALSENSE_SERIAL", c.RealSense.Serial)
		add("REALSENSE_DEPTH", c.RealSense.Depth)
	}
	add("CAPTURE", c.Capture)
	add("PIXEL_FORMAT", c.PixelFormat)
	add("OUTPUT", c.Output)
	if c.JPEGQuality != 0 {
		add("JPEG_QUALITY", strconv.Itoa(c.JPEGQuality))
	}
	add("ENCODER", c.Encoder)
	add("LISTEN", c.Listen)
	add("PATH_PREFIX", c.PathPrefix)
	add("NAME", c.Label)
	add("TITLE", c.Title)
	if v := c.Video; v != nil {
		address := v.Address
		if address == "tailscale" {
			address = cellDeployTailscaleAddress
		}
		add("VIDEO_ADDRESS", address)
		add("VIDEO_LISTEN", v.Listen)
		for _, kv := range []struct {
			key   string
			value int
		}{{"VIDEO_PORT", v.Port}, {"VIDEO_WIDTH", v.Width}, {"VIDEO_FPS", v.FPS}, {"VIDEO_KBPS", v.KBPS}} {
			if kv.value != 0 {
				add(kv.key, strconv.Itoa(kv.value))
			}
		}
	}
	return strings.Join(lines, "\n") + "\n"
}

// cellDeployCameraDefaultID is the id the OLP gives the cell camera in its
// camera list (offline-programming/v1 cell_camera.go); no extra camera's
// instance may be it.
const cellDeployCameraDefaultID = "default"

// validateExtraCameras: each extra camera is a full camera block without
// video, and no two cameras of the cell share an instance (one unit, one
// settings file), a device or a listen address.
func (c cellDeployConfig) validateExtraCameras() error {
	if len(c.ExtraCameras) == 0 {
		return nil
	}
	if c.Camera == nil {
		return fmt.Errorf("cell_deploy extra_cameras needs a cell_deploy.camera block: the cell camera is the one every client shows, the extras are listed beside it")
	}
	all := append([]cellDeployCamera{*c.Camera}, c.ExtraCameras...)
	for i, camera := range all {
		if i > 0 {
			if err := camera.validate(); err != nil {
				return fmt.Errorf("extra_cameras[%d]: %w", i-1, err)
			}
			if camera.Instance == cellDeployCameraDefaultID {
				return fmt.Errorf("cell_deploy extra_cameras[%d].instance %q is the cell camera's id in the OLP's list; choose another", i-1, camera.Instance)
			}
			// The OLP relays only the cell camera's video, and every sender
			// takes UDP 40400 by default: an extra camera's video would
			// run, unreachable, or fail to bind.
			if camera.Video != nil {
				return fmt.Errorf("cell_deploy extra_cameras[%d] %q has a video block: only the cell camera's video is relayed; remove it", i-1, camera.Instance)
			}
		}
		for _, other := range all[:i] {
			for field, pair := range map[string][2]string{"instance": {camera.Instance, other.Instance}, "camera": {camera.Camera(), other.Camera()}, "listen": {camera.Listen, other.Listen}} {
				if pair[0] == pair[1] {
					return fmt.Errorf("cell_deploy cameras %q and %q share %s %q: each camera has its own", other.Instance, camera.Instance, field, pair[0])
				}
			}
		}
	}
	return nil
}

// ExtraCamerasEnv is the OLP unit's OFFLINE_PROGRAMMING_CELL_EXTRA_CAMERAS
// value: <instance>=<origin>, comma-separated, in manifest order.
func (c cellDeployConfig) ExtraCamerasEnv() string {
	entries := make([]string, len(c.ExtraCameras))
	for i, camera := range c.ExtraCameras {
		entries[i] = camera.Instance + "=" + camera.Origin()
	}
	return strings.Join(entries, ",")
}

// CameraOrigin is what the OLP unit relays as the cell camera: the camera this
// manifest installs, else a camera service managed outside it, else none.
func (c cellDeployConfig) CameraOrigin() string {
	if c.Camera != nil {
		return c.Camera.Origin()
	}
	return c.OLP.CameraOrigin
}

const cellDeployCameraUsage = `usage: camera <install|status> --manifest cell.json [flags]

The cell camera service (cell-camera/v1) on the cell's relay; every value
comes from the manifest's cell_deploy.camera block, or with --camera
<instance> from that camera's block in cell_deploy.extra_cameras (another
angle of the same cell). Touches no controller and is not a robot action: it
needs no cell lock and sets no maintenance notice.

  install   Install the service from a built tree and (re)start the camera:
            --deployment <commit|dir> (default: the tree the OLP unit runs)
            Stops the units named in camera.replaces, renders the settings
            file, runs the tree's cell-camera/v1/host/install.sh, then reads
            status and one frame. A failed check stops the camera and starts
            the replaced units again (exit 5).
  status    Read-only: the unit, its settings, status and one frame
Both take --camera <instance>, --transport tailscale|ssh and --dry-run.
`

func runCamera(args []string, stdout, stderr io.Writer) error {
	if len(args) == 0 || isHelp(args[0]) {
		fmt.Fprint(stdout, cellDeployCameraUsage)
		return nil
	}
	switch args[0] {
	case "install":
		return runCellDeployCameraInstall(args[1:], stdout, stderr)
	case "status":
		return runCellDeployCameraStatus(args[1:], stdout, stderr)
	}
	return fmt.Errorf("unknown camera command %q", args[0])
}

// cellDeployCameraOf is the camera `--camera <instance>` names: the cell
// camera when instance is empty, else the camera of that instance.
func cellDeployCameraOf(ctx cellDeployContext, instance string) (cellDeployCamera, error) {
	if ctx.Deploy.Camera == nil {
		return cellDeployCamera{}, fmt.Errorf("cell %s names no camera: add a cell_deploy.camera block (steamdeck/real/v4/README.md, \"Adding a new robot cell\")", ctx.CellID)
	}
	if instance == "" || instance == ctx.Deploy.Camera.Instance {
		return *ctx.Deploy.Camera, nil
	}
	names := []string{ctx.Deploy.Camera.Instance}
	for _, camera := range ctx.Deploy.ExtraCameras {
		if camera.Instance == instance {
			return camera, nil
		}
		names = append(names, camera.Instance)
	}
	return cellDeployCamera{}, fmt.Errorf("cell %s has no camera %q: its cameras are %s", ctx.CellID, instance, strings.Join(names, ", "))
}

func runCellDeployCameraInstall(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var deployment, instance string
	ctx, restore, err := cellDeploySetup("camera install", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&deployment, "deployment", "", "built tree: a commit of this checkout or the deployment directory (default: the tree the OLP unit runs)")
		fs.StringVar(&instance, "camera", "", "which camera, by its instance (default: the cell camera, cell_deploy.camera)")
	}, &common, true, "--deployment", "--camera")
	if err != nil {
		return err
	}
	defer restore()
	camera, err := cellDeployCameraOf(ctx, instance)
	if err != nil {
		return err
	}
	dir := ""
	if deployment != "" {
		if dir, err = cellDeployResolveDeployment(ctx, deployment); err != nil {
			return err
		}
	}
	script := cellDeployCameraInstallScript(ctx, camera, dir)
	if common.DryRun {
		fmt.Fprintf(stdout, "camera install cell=%s unit=%s origin=%s (dry run: nothing contacted)\n# settings file\n%s# relay script\n%s\n",
			ctx.CellID, camera.Unit(), camera.Origin(), camera.Settings(), script)
		return nil
	}
	p := newRTCoreProgress(stdout, nil)
	p.Step("camera", fmt.Sprintf("camera install cell=%s relay=%s@%s unit=%s", ctx.CellID, ctx.RelayUser, ctx.RelayPeer, camera.Unit()))
	code, err := rtCoreConfigRunRemote(rtCoreConfigRemoteArgv(common.Transport, ctx.RelayUser, ctx.RelayPeer, ctx.RelayHost, script), p)
	return sessionOutcome("camera install", code, nil, nil, err)
}

func runCellDeployCameraStatus(args []string, stdout, stderr io.Writer) error {
	var common cellDeployCommonFlags
	var instance string
	ctx, restore, err := cellDeploySetup("camera status", args, stderr, func(fs *flag.FlagSet) {
		fs.StringVar(&instance, "camera", "", "which camera, by its instance (default: the cell camera, cell_deploy.camera)")
	}, &common, true, "--camera")
	if err != nil {
		return err
	}
	defer restore()
	camera, err := cellDeployCameraOf(ctx, instance)
	if err != nil {
		return err
	}
	script := cellDeployCameraStatusScript(camera)
	if common.DryRun {
		fmt.Fprintf(stdout, "camera status cell=%s unit=%s (read-only; dry run: nothing contacted)\n%s\n", ctx.CellID, camera.Unit(), script)
		return nil
	}
	p := newRTCoreProgress(stdout, nil)
	code, err := rtCoreConfigRunRemote(rtCoreConfigRemoteArgv(common.Transport, ctx.RelayUser, ctx.RelayPeer, ctx.RelayHost, script), p)
	return sessionOutcome("camera status", code, nil, nil, err)
}

// cellDeployCameraProbe reads status and one frame from the camera and prints
// what it found; it exits non-zero unless the frame is a JPEG. Unverified
// bound: a camera's first frame took 3.3 s on the first camera host, so 15 s covers a
// start.
const cellDeployCameraProbe = `probe() {
  python3 - "$ORIGIN" <<'ROSIE_CAMERA_PROBE'
import json, sys, time, urllib.request
origin = sys.argv[1]
deadline = time.time() + 15
problem = "no answer"
while time.time() < deadline:
    try:
        with urllib.request.urlopen(origin + "/frame.jpg?after=0", timeout=6) as reply:
            frame, sequence = reply.read(), reply.headers.get("X-Frame-Sequence")
        if frame.startswith(b"\xff\xd8") and frame.endswith(b"\xff\xd9"):
            with urllib.request.urlopen(origin + "/status", timeout=4) as reply:
                status = json.load(reply)
            print("camera frame %d bytes sequence %s" % (len(frame), sequence))
            print("camera status %s" % json.dumps({k: status.get(k) for k in ("camera", "resolution", "capture_resolution", "configured_fps", "encoding", "jpeg_quality", "active", "error")}))
            sys.exit(0)
        problem = "the answer is not a JPEG frame"
    except Exception as error:
        problem = str(error)
    time.sleep(0.5)
try:
    with urllib.request.urlopen(origin + "/status", timeout=4) as reply:
        problem += "; status error: %s" % json.load(reply).get("error")
except Exception:
    pass
print("camera no frame from %s: %s" % (origin, problem))
sys.exit(1)
ROSIE_CAMERA_PROBE
}
`

// cellDeployCameraInstallScript runs on the relay. dir is the built tree, or
// "" for the tree the OLP unit runs.
func cellDeployCameraInstallScript(ctx cellDeployContext, camera cellDeployCamera, dir string) string {
	return "set -uo pipefail\nexport LC_ALL=C\n" + cellDeployCameraProbe + rtCoreConfigReplace(`UNIT=@UNIT@; INSTANCE=@INSTANCE@; ORIGIN=@ORIGIN@; T=@TREE@; REPLACES=@REPLACES@; VIDEO_UNIT=@VIDEO_UNIT@; VIDEO_ORIGIN=@VIDEO_ORIGIN@; SDK=@SDK@
step() { printf '%s\n' "$*"; }
refuse() { printf '@@refused %s\n' "$*"; step "refused: $*"; exit 2; }
if [ -z "$T" ]; then
  wd=$(systemctl show @OLP_UNIT@ -p WorkingDirectory --value 2> /dev/null)
  T=${wd%/source/offline-programming/v1}
  [ -n "$T" ] && [ "$T" != "$wd" ] || refuse "the OLP unit names no deployment tree; pass --deployment"
fi
C="$T/source/cell-camera/v1"
[ -f "$C/host/install.sh" ] || refuse "$T has no cell-camera/v1 (a tree built before the camera service); build a newer commit and pass --deployment"
sudo -n true 2> /dev/null || refuse "the relay user cannot sudo without a password"
settings=$(mktemp) || refuse "mktemp failed"
trap 'rm -f "$settings"' EXIT
printf '%s' @SETTINGS_B64@ | base64 -d > "$settings" || refuse "settings did not decode"
if grep -q '@TAILSCALE_IPV4@' "$settings"; then
  ts=$(tailscale ip -4 2> /dev/null | head -n 1)
  [ -n "$ts" ] || refuse "the camera's video address is \"tailscale\" but tailscale ip -4 answered nothing on this relay"
  sed -i "s/@TAILSCALE_IPV4@/$ts/" "$settings"
  step "camera video address $ts (tailscale ip -4)"
fi
step "camera tree $C"
stopped=""
for u in $REPLACES; do
  if systemctl is-active --quiet "$u" || systemctl is-enabled --quiet "$u" 2> /dev/null; then
    sudo -n systemctl disable --now "$u" > /dev/null 2>&1 && stopped="$stopped $u" && step "camera replaced unit $u stopped and disabled"
  fi
done
# What runs now, to put back if this install fails: the selected package
# revision, this instance's settings, and whether the camera and its video
# sender were running. A failed install on a Rosie 1430 cell (2026-10-19) that only
# disabled the new unit left the cell with no camera.
STORE=/opt/rosie-cell-camera; SENDER="rosie-cell-video@$INSTANCE.service"
prev=$(readlink "$STORE/current" 2> /dev/null || true)
bak=$(mktemp) || refuse "mktemp failed"
trap 'rm -f "$settings" "$bak"' EXIT
sudo -n cat "/etc/rosie-cell-camera/$INSTANCE.env" > "$bak" 2> /dev/null || : > "$bak"
was_active=$(systemctl is-active --quiet "$UNIT" && echo yes || true)
restore() {
  if [ -n "$prev" ] && [ -d "$STORE/$prev" ] && [ -s "$bak" ]; then
    sudo -n ln -sfn "$prev" "$STORE/current.new" && sudo -n mv -Tf "$STORE/current.new" "$STORE/current"
    sudo -n install -m 0644 "$bak" "/etc/rosie-cell-camera/$INSTANCE.env"
    if grep -q '^CELL_CAMERA_VIDEO_ADDRESS=.' "$bak" && [ -x "$STORE/$prev/bin/cell-video" ]; then
      sudo -n systemctl restart "$SENDER" > /dev/null 2>&1
    else
      sudo -n systemctl disable --now "$SENDER" > /dev/null 2>&1
    fi
    if [ -n "$was_active" ]; then
      sudo -n systemctl enable "$UNIT" > /dev/null 2>&1; sudo -n systemctl restart "$UNIT" > /dev/null 2>&1
    fi
    step "camera restored the previous revision $prev and its settings ($UNIT $(systemctl is-active "$UNIT" 2> /dev/null))"
  else
    sudo -n systemctl disable --now "$UNIT" > /dev/null 2>&1
    sudo -n systemctl disable --now "$SENDER" > /dev/null 2>&1
  fi
  for u in $stopped; do sudo -n systemctl enable --now "$u" > /dev/null 2>&1 && step "camera restored $u"; done
}
if ! out=$(sudo -n bash "$C/host/install.sh" "$INSTANCE" "$settings" --start ${SDK:+--realsense-sdk "$SDK"} 2>&1); then
  step "$out"; restore; printf '@@result {"status":"failed","reason":"install.sh failed"}\n'; exit 5
fi
step "$out"
if ! probe; then
  sudo -n journalctl -u "$UNIT" -n 15 --no-pager -o cat 2> /dev/null | sed 's/^/camera journal /'
  restore; printf '@@result {"status":"failed","reason":"the camera gave no frame"}\n'; exit 5
fi
step "camera unit $UNIT $(systemctl is-active "$UNIT") $(systemctl is-enabled "$UNIT" 2> /dev/null)"
if [ -n "$VIDEO_UNIT" ]; then
  if ! out=$(curl -fsS -m 5 "$VIDEO_ORIGIN/status" 2>&1); then
    step "camera video sender: $out"
    sudo -n journalctl -u "$VIDEO_UNIT" -n 15 --no-pager -o cat 2> /dev/null | sed 's/^/camera video journal /'
    restore; printf '@@result {"status":"failed","reason":"the video sender did not answer"}\n'; exit 5
  fi
  step "camera video unit $VIDEO_UNIT $(systemctl is-active "$VIDEO_UNIT") status $out"
fi
printf '@@result {"status":"ok","unit":"%s","origin":"%s","tree":"%s"}\n' "$UNIT" "$ORIGIN" "$T"
`, "@UNIT@", shellQuote(camera.Unit()), "@INSTANCE@", shellQuote(camera.Instance), "@ORIGIN@", shellQuote(camera.Origin()),
		"@VIDEO_UNIT@", shellQuote(camera.VideoUnit()), "@VIDEO_ORIGIN@", shellQuote(camera.VideoOrigin()),
		"@TREE@", shellQuote(dir), "@REPLACES@", shellQuote(strings.Join(camera.Replaces, " ")), "@OLP_UNIT@", shellQuote(ctx.Deploy.OLP.Unit),
		"@SETTINGS_B64@", shellQuote(base64.StdEncoding.EncodeToString([]byte(camera.Settings()))), "@SDK@", shellQuote(camera.SDKDir()))
}

// SDKDir is the relay's pyrealsense2 package camera install copies beside a
// RealSense's service, "" for a V4L2 camera.
func (c cellDeployCamera) SDKDir() string {
	if c.RealSense == nil {
		return ""
	}
	return c.RealSense.SDKDir
}

// cellDeployCameraStatusScript is read-only apart from asking the camera for
// one frame, which starts a capture that stops by itself.
func cellDeployCameraStatusScript(camera cellDeployCamera) string {
	return "set -uo pipefail\nexport LC_ALL=C\n" + cellDeployCameraProbe + rtCoreConfigReplace(`UNIT=@UNIT@; ORIGIN=@ORIGIN@
echo "unit $UNIT $(systemctl is-active "$UNIT" 2> /dev/null) $(systemctl is-enabled "$UNIT" 2> /dev/null)"
for u in @REPLACES@; do echo "replaced $u $(systemctl is-active "$u" 2> /dev/null) $(systemctl is-enabled "$u" 2> /dev/null)"; done
echo "installed $(readlink /opt/rosie-cell-camera/current 2> /dev/null || echo none)"
if [ -n @VIDEO_UNIT@ ]; then
  echo "video unit "@VIDEO_UNIT@" $(systemctl is-active @VIDEO_UNIT@ 2> /dev/null) $(systemctl is-enabled @VIDEO_UNIT@ 2> /dev/null)"
  echo "video status $(curl -fsS -m 5 @VIDEO_ORIGIN@/status 2>&1 | head -c 600)"
fi
sudo -n cat /etc/rosie-cell-camera/@INSTANCE_NAME@.env 2> /dev/null | grep -v '^#' | sed 's/^/setting /'
probe || exit 1
`, "@UNIT@", shellQuote(camera.Unit()), "@ORIGIN@", shellQuote(camera.Origin()), "@REPLACES@", shellQuote(strings.Join(camera.Replaces, " ")),
		"@INSTANCE_NAME@", camera.Instance, "@VIDEO_UNIT@", shellQuote(camera.VideoUnit()), "@VIDEO_ORIGIN@", shellQuote(camera.VideoOrigin()))
}
