package benchsim

import (
	"bufio"
	"bytes"
	"encoding/json"
	"fmt"
	"math"
	"net"
	"strings"
	"time"
)

// The firmware's command layer: cmd_tier.c's table, cmd_gate.c's gates and the transport's
// per-connection state machine. WithFirmware turns it on; the website demo keeps its narrower
// command set without it.

const (
	flagNoHW = 1 << iota
	flagAnalog
	flagStream
	flagHeavy
	flagNoisy
	flagMixed
)

type fwRow struct {
	tier  int
	flags int
	fn    func(s *Sim, c *conn, r request) (any, error)
	raw   func(s *Sim, c *conn, r request) error // sends its own replies
}

func simple(h func(*Sim, request) (any, error)) func(*Sim, *conn, request) (any, error) {
	return func(s *Sim, _ *conn, r request) (any, error) { return h(s, r) }
}

// fwTable is k_tiers: every verb with its tier and gate flags. A row without a handler is a real
// command the simulator does not model: it gets a refusal saying so.
var fwTable map[string]fwRow

func init() {
	fwTable = map[string]fwRow{
		"ping":             {0, flagNoHW | flagNoisy, simple(func(*Sim, request) (any, error) { return "pong", nil }), nil},
		"status":           {0, flagNoHW, (*Sim).fwStatus, nil},
		"cloud_status":     {0, flagNoHW, nil, nil},
		"wifi_status":      {0, flagNoHW, nil, nil},
		"wifi_scan":        {0, flagNoHW, nil, nil},
		"la_pins":          {0, 0, simple((*Sim).cmdLaPins), nil},
		"usb_cc":           {0, flagNoHW, simple((*Sim).cmdUsbCc), nil},
		"target_status":    {0, flagNoHW | flagNoisy, simple((*Sim).cmdTargetStatus), nil},
		"power_status":     {0, flagNoHW | flagNoisy, simple((*Sim).cmdPowerStatus), nil},
		"identity_public":  {0, flagNoHW, nil, nil},
		"identity_pop":     {0, flagNoHW, nil, nil},
		"identity_wipe":    {0, flagNoHW, nil, nil},
		"spi_status":       {0, 0, simple((*Sim).cmdSpiStatus), nil},
		"sensor_status":    {0, 0, simple((*Sim).cmdSensorStatus), nil},
		"sensor_types":     {0, 0, simple((*Sim).cmdSensorTypes), nil},
		"gps_status":       {0, 0, simple((*Sim).cmdGpsStatus), nil},
		"can_status":       {0, flagNoHW, simple((*Sim).cmdCanStatus), nil},
		"ota_status":       {0, 0, nil, nil},
		"blob_status":      {0, 0, nil, nil},
		"dac_loop_probe":   {0, flagAnalog, (*Sim).cmdLoopProbe, nil},
		"capture":          {0, flagAnalog | flagStream | flagHeavy, nil, (*Sim).cmdCapture},
		"capture_dual":     {0, flagStream | flagHeavy, nil, (*Sim).cmdCaptureDual},
		"capture_read":     {0, flagStream | flagHeavy, nil, (*Sim).cmdCaptureRead},
		"stream":           {0, flagAnalog | flagStream | flagHeavy, nil, (*Sim).cmdCapture},
		"la_capture":       {0, flagStream | flagHeavy, nil, (*Sim).cmdLaCapture},
		"sensor_regs":      {0, flagStream | flagHeavy, nil, (*Sim).cmdSensorRegs},
		"sensor_la":        {0, flagStream | flagHeavy, nil, (*Sim).cmdSensorLa},
		"can_read":         {0, flagNoHW | flagHeavy, simple((*Sim).cmdCanRead), nil},
		"psram_ping":       {0, flagHeavy, (*Sim).cmdPsramPing, nil},
		"test":             {0, flagStream | flagHeavy, nil, nil},
		"dac_limits":       {0, flagNoHW | flagAnalog | flagMixed, simple((*Sim).cmdDacLimits), nil},
		"calibrate":        {0, flagAnalog | flagMixed, (*Sim).cmdCalibrate, nil},
		"eth":              {0, flagNoHW | flagMixed, nil, nil},
		"sig_policy":       {0, flagMixed, (*Sim).cmdSigPolicy, nil},
		"lan_policy":       {0, flagMixed, (*Sim).cmdLanPolicy, nil},
		"usb_network":      {0, flagMixed, nil, nil},
		"module":           {0, flagNoHW | flagMixed, nil, nil},
		"cloud_ca":         {0, flagMixed, (*Sim).cmdCloudCa, nil},
		"cloud_proxy":      {0, flagMixed, (*Sim).cmdCloudProxy, nil},
		"lan_tls":          {0, flagMixed, nil, nil},
		"generate":         {1, flagAnalog, (*Sim).cmdGenerateFw, nil},
		"measure":          {1, flagAnalog | flagStream, nil, nil},
		"load":             {1, flagAnalog | flagStream, nil, nil},
		"load_bin":         {1, flagAnalog | flagStream, nil, (*Sim).cmdLoadBin},
		"replay":           {1, flagAnalog | flagStream, (*Sim).cmdReplay, nil},
		"dac_stop":         {1, 0, simple((*Sim).cmdDacStop), nil},
		"dac_set":          {1, flagAnalog, simple((*Sim).cmdDacSet), nil},
		"dac_mux":          {1, flagAnalog, nil, nil},
		"cal_switch":       {1, flagAnalog, nil, nil},
		"analog_path":      {1, flagAnalog, simple((*Sim).cmdAnalogPath), nil},
		"dac_out":          {1, flagAnalog, simple((*Sim).cmdDacOut), nil},
		"current_out":      {1, flagAnalog, simple((*Sim).cmdCurrentOut), nil},
		"adc_read":         {1, flagAnalog, (*Sim).cmdAdcReadFw, nil},
		"adc_range":        {1, flagAnalog, simple((*Sim).cmdAdcRange), nil},
		"dac_control_loop": {1, flagAnalog, (*Sim).cmdControlLoop, nil},
		"dac_loop_input":   {1, flagAnalog, (*Sim).cmdLoopInput, nil},
		"la":               {1, 0, simple((*Sim).cmdLa), nil},
		"gpio":             {1, 0, simple((*Sim).cmdGpio), nil},
		"la_voltage":       {1, flagNoHW, simple((*Sim).cmdLaVoltage), nil},
		"nrst":             {1, flagNoHW, simple((*Sim).cmdNrst), nil},
		"target_power":     {1, flagNoHW, simple((*Sim).cmdTargetPower), nil},
		"module_power":     {1, 0, nil, nil},
		"power_profile":    {1, flagNoHW, nil, (*Sim).cmdPowerProfile},
		"dap_start":        {1, flagStream, nil, nil},
		"uart_proxy_start": {1, flagStream, nil, (*Sim).cmdUartProxy},
		"spi_start":        {1, 0, simple((*Sim).cmdSpiStart), nil},
		"spi_stop":         {1, 0, simple((*Sim).cmdSpiStop), nil},
		"spi_xfer":         {1, 0, simple((*Sim).cmdSpiXfer), nil},
		"spi_stream":       {1, 0, (*Sim).cmdSpiStream, nil},
		"spi_flash":        {1, 0, simple((*Sim).cmdSpiFlash), nil},
		"sensor_start":     {1, 0, simple((*Sim).cmdSensorStart), nil},
		"sensor_set":       {1, 0, simple((*Sim).cmdSensorSet), nil},
		"sensor_stop":      {1, 0, simple((*Sim).cmdSensorStop), nil},
		"gps_start":        {1, 0, simple((*Sim).cmdGpsStart), nil},
		"gps_set":          {1, 0, simple((*Sim).cmdGpsSet), nil},
		"gps_stop":         {1, 0, simple((*Sim).cmdGpsStop), nil},
		"can_config":       {1, flagNoHW, simple((*Sim).cmdCanConfig), nil},
		"can_write":        {1, flagNoHW, simple((*Sim).cmdCanWrite), nil},
		"can_term":         {1, flagNoHW, simple((*Sim).cmdCanTerm), nil},
		"can_respond":      {1, flagNoHW, simple((*Sim).cmdCanRespond), nil},
		"can_disable":      {1, flagNoHW, simple((*Sim).cmdCanDisable), nil},
		"speedtest":        {1, flagNoHW | flagStream, nil, nil},
		"fpga_image":       {1, 0, (*Sim).cmdFpgaImage, nil},
		"psram_recover":    {1, 0, simple((*Sim).cmdPsramRecover), nil},
		"cloud_set":        {2, flagNoHW, nil, nil},
		"cloud_clear":      {2, flagNoHW, nil, nil},
		"wifi_set":         {2, flagNoHW, nil, nil},
		"wifi_clear":       {2, flagNoHW, nil, nil},
		"ota_begin":        {3, 0, nil, nil},
		"ota_data":         {3, 0, nil, nil},
		"ota_end":          {3, 0, nil, nil},
		"ota_commit":       {3, 0, nil, nil},
		"ota_abort":        {3, 0, nil, nil},
		"ota_selftest":     {3, 0, nil, nil},
	}
}

// mixedWrite is cmd_tier.c's mixed_write: the write forms of the mixed verbs are T2.
func mixedWrite(cmd string, r request) bool {
	notFalse := func(k string) bool { return r.has(k) && r.str(k) != "false" }
	switch cmd {
	case "dac_limits":
		return r.has("path") || (r.has("enabled") && r.str("enabled") == "false")
	case "calibrate":
		return r.has("source") || r.has("dac") || r.has("adc") || notFalse("clear")
	case "sig_policy", "lan_policy", "usb_network":
		return r.has("set")
	case "cloud_ca", "cloud_proxy":
		return r.has("set") || r.str("clear") == "true"
	case "lan_tls":
		return r.has("https_only") || r.str("regenerate") == "true"
	case "module":
		return r.has("eeprom") && r.str("eeprom") != "read"
	case "eth":
		a := r.str("action")
		return a != "stats" && a != "refclk"
	}
	return false
}

func tierOf(cmd string, row fwRow, known bool, r request) int {
	if !known {
		return 3
	}
	if row.flags&flagMixed != 0 && mixedWrite(cmd, r) {
		return 2
	}
	return row.tier
}

// hostCommands are the verbs the cloud-link side of the pod owns (the OTA session, the link state).
var hostCommands = map[string]bool{
	"cloud_status": true, "ota_begin": true, "ota_data": true, "ota_end": true, "ota_commit": true,
	"ota_abort": true, "ota_status": true, "ota_selftest": true,
}

// fwDispatch is dispatch_line: the gates, the DAC limits, then the handler.
func (s *Sim) fwDispatch(c *conn, r request) {
	cmd := r.str("cmd")
	if cmd == "" {
		_ = c.send(errReply("missing cmd"))
		return
	}
	row, known := fwTable[cmd]
	tier := tierOf(cmd, row, known, r)
	note := func(err error) {
		s.logCmd(r, err)
		if s.hooks.Logged != nil && !c.single {
			s.hooks.Logged(map[string]any(r), c.cloud, err)
		}
	}
	fail := func(err error) {
		note(err)
		_ = c.send(errReply(err.Error()))
	}
	s.mu.Lock()
	lanPolicy := s.flash.lanPolicy
	held, holder, left := s.leaseLocked()
	s.mu.Unlock()
	if !c.cloud {
		if tier >= 2 && lanPolicy != "open" {
			fail(errf("locked: %s needs the cloud or USB", cmd))
			return
		}
		light := known && row.flags&flagHeavy == 0 && tier == 0
		if held && !light {
			if holder == "" {
				holder = "cloud"
			}
			fail(errf("busy: a cloud job holds this pod (%s, %d s left)", holder, left))
			return
		}
	}
	if c.maxTier >= 0 && tier > c.maxTier {
		fail(errf("forbidden: %s needs an organization owner or admin", cmd))
		return
	}
	if c.single && row.flags&flagStream != 0 {
		fail(fwError("command not supported over cloud channel"))
		return
	}
	s.mu.Lock()
	why := s.dacLimitsCheck(cmd, r)
	s.mu.Unlock()
	if why != "" {
		fail(fwError(why))
		return
	}
	if hostCommands[cmd] && s.hooks.Command != nil {
		if data, msg, ok := s.hooks.Command(cmd, map[string]any(r), c.cloud); ok {
			if msg != "" {
				fail(fwError(msg))
				return
			}
			note(nil)
			_ = c.send(okReply(data))
			return
		}
	}
	switch {
	case !known:
		fail(fwError("unknown cmd"))
	case row.raw != nil:
		note(row.raw(s, c, r))
	case row.fn != nil:
		data, err := row.fn(s, c, r)
		note(err)
		if err != nil {
			_ = c.send(errReply(err.Error()))
			return
		}
		_ = c.send(okReply(data))
	default:
		fail(errf("%s is not modeled by the simulator", cmd))
	}
}

// leaseLocked is lease_gate_active. Callers hold s.mu.
func (s *Sim) leaseLocked() (held bool, holder string, leftS int) {
	l := s.pod.lease
	if !l.held || !time.Now().Before(l.until) {
		return false, "", 0
	}
	return true, l.holder, int(time.Until(l.until).Seconds())
}

// SetLease is the pod's lease.state frame.
func (s *Sim) SetLease(held bool, holder string, expiresIn time.Duration) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if !held {
		s.pod.lease = leaseState{}
		return
	}
	s.pod.lease = leaseState{held: true, holder: holder, until: time.Now().Add(expiresIn)}
}

// ServeTunnel runs a cloud tunnel: the same protocol as a LAN connection, with the tier limit
// the server sent in tunnel.open. It returns when c closes.
func (s *Sim) ServeTunnel(c net.Conn, maxTier int) {
	s.wg.Add(1)
	s.mu.Lock()
	s.conns[c] = struct{}{}
	s.mu.Unlock()
	s.serveStream(c, true, maxTier)
}

// serveStream is command_handler_process for one connection: the first byte picks JSON or SCPI,
// and lines are commands until a handler takes the connection over.
func (s *Sim) serveStream(c net.Conn, cloud bool, maxTier int) {
	defer s.wg.Done()
	cn := &conn{Conn: c, br: bufio.NewReaderSize(c, 8192), sim: s, cloud: cloud, maxTier: maxTier}
	defer func() {
		s.mu.Lock()
		delete(s.conns, c)
		s.heavyRelease(cn)
		s.loadAbandon(cn)
		s.mu.Unlock()
		_ = c.Close()
	}()
	if tc, ok := c.(*net.TCPConn); ok {
		_ = tc.SetNoDelay(true)
	}
	for {
		line, err := readLine(cn.br, 1<<20)
		if err != nil {
			return
		}
		if len(bytes.TrimSpace(line)) == 0 {
			continue
		}
		if t := bytes.TrimLeft(line, " \t"); t[0] != '{' {
			s.scpiLine(cn, string(t))
			continue
		}
		var req request
		if json.Unmarshal(line, &req) != nil || req == nil {
			_ = cn.send(errReply("invalid json"))
			continue
		}
		s.fwDispatch(cn, req)
		if cn.closed {
			return
		}
	}
}

// scpiLine answers the SCPI queries clients use to find the instrument; a setter is refused
// silently while a cloud job holds the pod, like scpi_line_blocked_by_lease.
func (s *Sim) scpiLine(c *conn, line string) {
	if strings.EqualFold(strings.TrimSpace(line), "*IDN?") {
		_, _ = c.Write([]byte(fmt.Sprintf("EmbeddedCI,BenchPod,0,%s\r\n", s.firmwareVersion())))
	}
}

func (s *Sim) firmwareVersion() string {
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.fwVersion != "" {
		return s.fwVersion
	}
	return podFirmware
}

// SetFirmwareVersion is what a committed OTA changes.
func (s *Sim) SetFirmwareVersion(v string) {
	s.mu.Lock()
	s.fwVersion = v
	s.mu.Unlock()
}

// bufConn captures what a handler writes, for the single-reply cloud command channel.
type bufConn struct {
	bytes.Buffer
}

func (b *bufConn) Close() error                     { return nil }
func (b *bufConn) LocalAddr() net.Addr              { return nil }
func (b *bufConn) RemoteAddr() net.Addr             { return nil }
func (b *bufConn) SetDeadline(time.Time) error      { return nil }
func (b *bufConn) SetReadDeadline(time.Time) error  { return nil }
func (b *bufConn) SetWriteDeadline(time.Time) error { return nil }

// Exec runs one command as a single-reply caller: over the cloud command channel (cloud true,
// streaming verbs refused) or as a LAN client. The data is the reply's data, or errMsg its refusal.
func (s *Sim) Exec(req map[string]any, cloud bool) (data json.RawMessage, errMsg string) {
	s.mu.Lock()
	cn := s.cmdConn
	s.mu.Unlock()
	buf := &bufConn{}
	c := &conn{Conn: buf, br: bufio.NewReader(buf), sim: s, cloud: cloud, maxTier: -1, single: cloud, heavyID: cn}
	s.fwDispatch(c, request(req))
	s.mu.Lock()
	s.heavyRelease(c)
	s.mu.Unlock()
	line, _, _ := bytes.Cut(buf.Bytes(), []byte{'\n'})
	var r struct {
		Status  string          `json:"status"`
		Message string          `json:"message"`
		Data    json.RawMessage `json:"data"`
	}
	if json.Unmarshal(line, &r) != nil {
		return nil, "reply too large for the cloud channel"
	}
	if r.Status == "error" {
		return nil, r.Message
	}
	return r.Data, ""
}

// heavyBegin is heavy_begin: one owner of the capture hardware at a time. Callers hold s.mu.
func (s *Sim) heavyBegin(c *conn) error {
	if s.heavyInFlight() || (s.pod.heavy != nil && s.pod.heavy != c.heavyKey()) {
		return fwError("busy")
	}
	s.pod.heavy = c.heavyKey()
	return nil
}

func (s *Sim) heavyRelease(c *conn) {
	if s.pod.heavy == c.heavyKey() {
		s.pod.heavy = nil
	}
}

func (s *Sim) heavyInFlight() bool { return s.pod.capture || s.pod.otaBusy }

// SetOTABusy marks an OTA staging or verified image: it owns the PSRAM bus (heavy_in_flight).
func (s *Sim) SetOTABusy(busy bool) {
	s.mu.Lock()
	s.pod.otaBusy = busy
	s.mu.Unlock()
}

// CaptureOrUploadBusy is capture_or_upload_busy, which ota_begin refuses on.
func (s *Sim) CaptureOrUploadBusy() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.pod.capture || s.pod.heavy != nil || s.pod.load != nil
}

// fwStatus is handle_status. The cloud command channel leaves out the ADC fit: cloud clients get
// it from the capabilities frame.
func (s *Sim) fwStatus(c *conn, _ request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	held, holder, left := s.leaseLocked()
	version := s.fwVersion
	if version == "" {
		version = podFirmware
	}
	out := map[string]any{
		"device": "benchpod", "version": version, "board": podBoard, "net": "connected", "ip": "127.0.0.1",
		"rssi_dbm": nil, "wifi": "off", "cloud": s.cloudState(),
		"usb":        map[string]any{"net": "off", "setting": "on", "off_reason": "wifi", "link": false, "ip": "", "host_ip": "", "source": "3.0A", "source_ma": 3000},
		"rx_dropped": 0, "tx_dropped": 0, "adc_bits": 16, "adc_fullscale_mv": 4096, "adc_channels": 1,
		"la_vccio_mv": s.pod.laMV, "board_rev": "v3", "board_rev_mv": 1650, "nrst_pin": true, "flash_kb": 2048,
		"ota_sig": true, "sig_policy": s.flash.sigPolicy, "sig_keys": 1, "sig_policy_cmd": true,
		"lan_policy": s.flash.lanPolicy, "lan_policy_cmd": true, "tunnel_max_tier": true, "lease_state": true,
		"cloud_ca": true, "cloud_proxy": true,
		"lan_tls":           map[string]any{"port": 8443, "https_only": false, "sha256": ""},
		"lease":             map[string]any{"held": held, "holder": holder, "left_s": left},
		"gateware":          podGateware,
		"gateware_embedded": podGateware, "loop_tripped": s.loopTripped(), "uart_rx_overflow": false,
		"psram": "ok", "psram_ok": true, "heap_free": 120000, "heap_min": 90000, "stack_min": 800,
		"reset": "power-on", "last_crash": "none", "stacks": map[string]any{}, "analog": true,
		"safe_mode": false, "safe_reason": "", "caps": s.capsList(),
	}
	if !c.single {
		out["adc_cal_a_uv"] = int64(math.Round(adcCalA * 1e6))
		out["adc_cal_b_nv"] = int64(math.Round(adcCalB * 1e9))
		out["adc_cal_unwrap"] = true
		out["adc_range"] = "normal"
		out["adc_range_fs_mv"] = adcFullscaleMv()
	}
	return out, nil
}

func (s *Sim) cloudState() string {
	if s.hooks.Command == nil {
		return "off"
	}
	return "connected"
}

func adcFullscaleMv() int64 { return int64(math.Round(math.Abs(adcCalB) * 32768 * 1000)) }

// capsList is cloud_caps_emit_list for an analog board on the running gateware image.
func (s *Sim) capsList() []string {
	caps := []string{"signal", "gpio", "power", "swd", "i2c_sensor", "uart", "la", "analyzer", "command",
		"tunnel", "ota", "la_pins", "power_profile", "capture_b64", "can", "sensor_types", "usb_net",
		"usb_net_setting", "lan_tls", "wifi_scan", "pod_current", "analog", "scope", "dac_limits", "calibrate",
		"cal_dmm", "current_out", "dac", "dac_dc", "dac_replay"}
	if s.pod.image == 1 {
		caps = append(caps, "dac_deep_replay")
	} else {
		caps = append(caps, "dac_control_loop")
	}
	return append(caps, "dac_cotrig", "generate16", "dac_loop_sources", "dac_loop_input_map", "gpio_read",
		"capture_trigger", "spi_master", "spi_stream", "gps", "nrst_pin", "usb_cc")
}

// Capabilities is the firmware's capabilities frame (cloud_caps.c), without device_id and the
// boot health the link side adds.
func (s *Sim) Capabilities() map[string]any {
	s.mu.Lock()
	defer s.mu.Unlock()
	version := s.fwVersion
	if version == "" {
		version = podFirmware
	}
	deep := s.pod.image == 1
	maxSamples := 2048
	if deep {
		maxSamples = 4 << 20
	}
	return map[string]any{
		"firmware_version": version, "ota": true, "flash_kb": 2048, "blob_slots": true, "ota_sig": true,
		"sig_policy": s.flash.sigPolicy, "sig_policy_cmd": true, "ws_auth_v2": true, "lease_state": true,
		"cloud_ca": true, "cloud_proxy": true, "lan_policy": s.flash.lanPolicy, "lan_policy_cmd": true,
		"tunnel_max_tier": true, "lan_tls": true, "wifi_scan": true, "serial": false, "scope": true, "analog": true,
		"analyzer": true, "command": true, "tunnel": true, "adc_bits": 16, "adc_fullscale_mv": 4096, "adc_channels": 1,
		"adc_cal_a_uv": int64(math.Round(adcCalA * 1e6)), "adc_cal_b_nv": int64(math.Round(adcCalB * 1e9)),
		"adc_cal_unwrap": true, "adc_low_range": false, "adc_range": "normal", "adc_range_fs_mv": adcFullscaleMv(),
		"dac": true, "dac_replay": true, "dac_dc": true, "dac_bits": 8, "dac_replay_bits": 16,
		"dac_fullscale_mv": 4096, "dac_channels": 1, "dac_deep_replay": deep, "dac_replay_max_samples": maxSamples,
		"dac_control_loop": !deep, "dac_loop_sources": true, "dac_loop_input_map": true, "dac_cotrig": true,
		"generate_bits": 16, "la_pins": true, "gpio_read": true, "capture_trigger": true, "spi_master": true,
		"spi_stream": true, "nrst_pin": true, "power_profile": true, "capture_b64": true, "dac_limits": true,
		"calibrate": true, "cal_dmm": true, "can": true, "pod_current": true, "current_out": true,
		"current_out_min_ua": currentOutUa(0), "current_out_max_ua": currentOutUa(0xFFFF), "sensor_types": true,
		"gps": true, "board": podBoard, "uplink": "eth",
	}
}

// LAState reads the LA bank and rails for the link side and for tests.
func (s *Sim) LAState() (mv int, pulls uint16, efuse1, efuse2 bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.advance(s.now())
	return s.pod.laMV, s.pod.pulls, s.pod.efuse[1].enabled, s.pod.efuse[2].enabled
}

// EfuseState reads one rail's tracked state.
func (s *Sim) EfuseState(n int) (enabled, fault bool) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.advance(s.now())
	return s.pod.efuse[n].enabled, s.pod.efuse[n].fault
}

// Reboot is a pod reset: power, the LA bank, the fabric and every session gone; what the firmware
// keeps in flash stays. Open connections drop.
func (s *Sim) Reboot() {
	s.mu.Lock()
	now := s.now()
	s.advance(now)
	if s.pod.efuse[1].enabled {
		s.applyPower(now, 1, false)
	}
	s.pod.efuse[2].enabled = false
	s.pod = podState{}
	s.pod.init()
	s.touchBase(now)
	conns := make([]net.Conn, 0, len(s.conns))
	for c := range s.conns {
		conns = append(conns, c)
	}
	s.mu.Unlock()
	for _, c := range conns {
		_ = c.Close()
	}
}

// lanOff: a LAN connection under policy "off" is accepted and reset.
func (s *Sim) lanOff() bool {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.flash.lanPolicy == "off"
}

// SetCA records an installed company CA (the OTA target "ca").
func (s *Sim) SetCA(certs []map[string]any) {
	s.mu.Lock()
	s.flash.caCerts = certs
	s.mu.Unlock()
}

// ProxySetting is the stored cloud_proxy, for the link side.
func (s *Sim) ProxySetting() (host string, port int, user, pass string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.flash.proxyHost, s.flash.proxyPort, s.flash.proxyUser, s.flash.proxyPass
}
