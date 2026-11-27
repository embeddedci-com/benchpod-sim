package benchsim

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"net"
	"sort"
	"strconv"
	"strings"
)

// request is one decoded JSON command. The SDK sends numbers as JSON numbers, a few fields (an
// I2C address, a register) as hex strings, and some channel fields as a number or a list.
type request map[string]any

func (r request) has(k string) bool { _, ok := r[k]; return ok }

func (r request) str(k string) string {
	switch v := r[k].(type) {
	case string:
		return v
	case float64:
		return strconv.FormatFloat(v, 'f', -1, 64)
	case bool:
		return strconv.FormatBool(v)
	}
	return ""
}

// num reads a number, also from a decimal or 0x-hex string (the firmware's parser takes both).
func (r request) num(k string) (float64, bool) {
	switch v := r[k].(type) {
	case float64:
		return v, !math.IsNaN(v) && !math.IsInf(v, 0)
	case bool:
		if v {
			return 1, true
		}
		return 0, true
	case string:
		s := strings.TrimSpace(v)
		if strings.HasPrefix(strings.ToLower(s), "0x") {
			n, err := strconv.ParseInt(s[2:], 16, 64)
			return float64(n), err == nil
		}
		f, err := strconv.ParseFloat(s, 64)
		return f, err == nil
	}
	return 0, false
}

func (r request) int(k string, def int) int {
	if v, ok := r.num(k); ok {
		return int(v)
	}
	return def
}

func (r request) flag(k string) bool {
	v, ok := r.num(k)
	return ok && v != 0
}

// channels reads an "la" field: a number, a list of numbers or "all" (all=true).
func (r request) channels(k string) (las []int, all bool, err error) {
	switch v := r[k].(type) {
	case nil:
		return nil, false, errors.New("missing la")
	case string:
		if v == "all" {
			return nil, true, nil
		}
		n, e := strconv.Atoi(v)
		if e != nil {
			return nil, false, fmt.Errorf("invalid la %q", v)
		}
		return []int{n}, false, nil
	case float64:
		return []int{int(v)}, false, nil
	case []any:
		for _, x := range v {
			f, ok := x.(float64)
			if !ok {
				return nil, false, errors.New("la must be numbers")
			}
			las = append(las, int(f))
		}
		if len(las) == 0 {
			return nil, false, errors.New("la list is empty")
		}
		return las, false, nil
	}
	return nil, false, errors.New("invalid la")
}

func encodeLine(v any) []byte {
	b, err := json.Marshal(v)
	if err != nil {
		b = []byte(`{"status":"error","message":"reply encode failed"}`)
	}
	return append(b, '\n')
}

func okReply(data any) map[string]any { return map[string]any{"status": "ok", "data": data} }
func errReply(msg string) map[string]any {
	return map[string]any{"status": "error", "message": msg}
}

// errorReply is the reply line for a refusal: a coded one as {"status":"error","code":...,
// "args":{...}} (no message, firmware 4.0.0), any other as {"status":"error","message":...}.
func errorReply(err error) map[string]any {
	var ce *codedError
	if errors.As(err, &ce) {
		return ce.reply()
	}
	return errReply(err.Error())
}

// codedError is a refusal with a stable error code (bp_errors.c): the client renders the
// catalog's message from the code and its args.
type codedError struct {
	code int
	args map[string]any
}

// coded builds a codedError from key, value pairs.
func coded(code int, kv ...any) error {
	e := &codedError{code: code}
	if len(kv) > 0 {
		e.args = map[string]any{}
		for i := 0; i+1 < len(kv); i += 2 {
			e.args[kv[i].(string)] = kv[i+1]
		}
	}
	return e
}

// Code is the code as the pod writes it, "BP0031".
func (e *codedError) Code() string { return fmt.Sprintf("BP%04d", e.code) }

// Error is the firmware's short form, "BP0031 la=4 owner=gpio" (bp_err_render), keys sorted.
func (e *codedError) Error() string {
	keys := make([]string, 0, len(e.args))
	for k := range e.args {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	var b strings.Builder
	b.WriteString(e.Code())
	for _, k := range keys {
		fmt.Fprintf(&b, " %s=%v", k, e.args[k])
	}
	return b.String()
}

func (e *codedError) reply() map[string]any {
	m := map[string]any{"status": "error", "code": e.Code()}
	if len(e.args) > 0 {
		m["args"] = e.args
	}
	if e.code == codeClientTooOld { // the one coded refusal that keeps a message, for old clients
		m["message"] = fmt.Sprintf("Update the BenchPod SDK, MCP or CLI: this firmware needs client API %v or newer (this client sent %v)",
			e.args["min_client_api"], e.args["client_api"])
	}
	return m
}

// The error codes the simulator sends (errors/catalog.toml).
const (
	codeLaVoltageNotSet      = 1
	codeUnknownCommand       = 2
	codeLeaseBusy            = 25
	codeClientTooOld         = 30
	codeLaPinInUse           = 31
	codeLaPullDownEngaged    = 32
	codeLaVoltagePinsInUse   = 37
	codeLaPullDownConflict   = 33
	codeLanLocked            = 40
	codePolicyNotFromLan     = 41
	codeSettingCloudOrUsb    = 42
	codeSettingDeviceOnly    = 43
	codeRoleRequired         = 45
	codePodBusy              = 47
	codeStorageBusBusy       = 48
	codeSignaturesAlwaysReqd = 74
	codeTriggerTimeout       = 38
	codeSamplesOutOfRange    = 67
	codePsramTooSmall        = 72
	codeRebootHardNotOnPod   = 93
	codeLaNoPull             = 23
	codeLaPullDirection      = 24
)

// The client API level (features.toml): what this firmware speaks and the lowest it takes.
const (
	apiLevel     = 2
	minClientAPI = 2
	featuresRev  = 11
)

// anyClient are the verbs every client may send whatever its client_api (CMD_F_ANY_CLIENT), so
// an old client can still read the pod, let go of it and update it.
var anyClient = map[string]bool{
	"status": true, "diag": true, "ping": true, "lease_release": true, "ota_status": true,
	"ota_begin": true, "ota_data": true, "ota_end": true, "ota_commit": true, "ota_abort": true,
	"ota_selftest": true, "blob_status": true, "identity_public": true,
}

// clientAPIGate is cmd_gate_client_api: a command whose top-level client_api (none = 0) is below
// min_client_api is refused with BP0030, before every other gate.
func clientAPIGate(cmd string, r request) error {
	if anyClient[cmd] {
		return nil
	}
	n := 0
	switch v := r["client_api"].(type) {
	case float64:
		if v >= 0 && v == math.Trunc(v) {
			n = int(min(v, math.MaxInt32))
		}
	case int:
		n = max(v, 0)
	case string:
		if x, err := strconv.Atoi(v); err == nil && x >= 0 {
			n = x
		}
	}
	if n >= minClientAPI {
		return nil
	}
	return coded(codeClientTooOld, "client_api", n, "min_client_api", minClientAPI, "api", apiLevel)
}

// conn is one client connection: one request, then its reply (or a stream, or raw UART).
type conn struct {
	net.Conn
	br  *bufio.Reader
	sim *Sim
	// The firmware command layer (firmware.go): where the connection comes from, its tier limit
	// (-1 = none), single-reply (the cloud command channel), and heavyID, the gate owner when the
	// single-reply calls share one (the firmware's CH_CLOUD_CONN).
	cloud   bool
	maxTier int
	single  bool
	heavyID *conn
	closed  bool
	trigger map[string]any
}

func (c *conn) heavyKey() *conn {
	if c.heavyID != nil {
		return c.heavyID
	}
	return c
}

func (c *conn) send(v any) error {
	_, err := c.Write(encodeLine(v))
	return err
}

// fwError is a refusal worded like the firmware's {"status":"error","message":...}.
type fwError string

func (e fwError) Error() string { return string(e) }

func errf(format string, a ...any) error { return fwError(fmt.Sprintf(format, a...)) }

// notInDemo is the reply for a real pod command this simulator leaves out.
func notInDemo(cmd string) error {
	return errf("%s is not available in the demo simulator", cmd)
}

// simpleHandlers answer with one reply line. Streaming and mode-switching commands are handled
// in dispatch itself.
var simpleHandlers = map[string]func(s *Sim, r request) (any, error){
	"ping":                  func(*Sim, request) (any, error) { return "pong", nil },
	"status":                (*Sim).cmdStatus,
	"la_voltage":            (*Sim).cmdLaVoltage,
	"cloud_status":          func(*Sim, request) (any, error) { return map[string]any{"configured": false, "state": "off"}, nil },
	"lease_state":           func(*Sim, request) (any, error) { return map[string]any{"held": false}, nil },
	"target_power":          (*Sim).cmdTargetPower,
	"target_status":         (*Sim).cmdTargetStatus,
	"power_status":          (*Sim).cmdPowerStatus,
	"nrst":                  (*Sim).cmdNrst,
	"usb_cc":                (*Sim).cmdUsbCc,
	"la_step":               (*Sim).cmdLaStep,
	"la_pull":               (*Sim).cmdLaPull,
	"la_pins":               (*Sim).cmdLaPins,
	"gpio":                  (*Sim).cmdGpio,
	"sensor_define":         (*Sim).cmdSensorDefine,
	"sensor_start":          (*Sim).cmdSensorStart,
	"sensor_set":            (*Sim).cmdSensorSet,
	"sensor_stop":           (*Sim).cmdSensorStop,
	"sensor_status":         (*Sim).cmdSensorStatus,
	"sensor_regs":           (*Sim).cmdSensorRegs,
	"sensor_fault":          (*Sim).cmdSensorFault,
	"sensor_fault_clear":    (*Sim).cmdSensorFaultClear,
	"sensor_stimulus":       (*Sim).cmdSensorStimulus,
	"sensor_stimulus_start": (*Sim).cmdSensorStimulusStart,
	"sensor_stimulus_stop":  (*Sim).cmdSensorStimulusStop,
	"analog_path":           (*Sim).cmdAnalogPath,
	"dac_out":               (*Sim).cmdDacOut,
	"adc_read":              (*Sim).cmdAdcRead,
	"generate":              (*Sim).cmdGenerate,
	"dac_stop":              (*Sim).cmdDacStop,
}

func (s *Sim) dispatch(c *conn, req request) {
	if s.firmware {
		s.fwDispatch(c, req)
		return
	}
	cmd := req.str("cmd")
	if cmd == "" {
		err := fwError("missing cmd")
		s.logCmd(req, err)
		_ = c.send(errorReply(err))
		return
	}
	if err := clientAPIGate(cmd, req); err != nil {
		s.logCmd(req, err)
		_ = c.send(errorReply(err))
		return
	}
	if err := s.analogGuard(cmd); err != nil {
		s.logCmd(req, err)
		_ = c.send(errorReply(err))
		return
	}
	if h, ok := simpleHandlers[cmd]; ok {
		data, err := h(s, req)
		s.logCmd(req, err)
		if err != nil {
			_ = c.send(errorReply(err))
			return
		}
		_ = c.send(okReply(data))
		return
	}
	var err error
	switch cmd {
	case "uart_proxy_start":
		err = s.cmdUartProxy(c, req)
	case "la_capture":
		err = s.cmdLaCapture(c, req)
	case "sensor_la":
		err = s.cmdSensorLa(c, req)
	case "capture":
		err = s.cmdCapture(c, req)
	case "capture_dual":
		err = s.cmdCaptureDual(c, req)
	case "power_profile":
		err = s.cmdPowerProfile(c, req)
	default:
		err = notInDemo(cmd)
		if !knownPodCommands[cmd] {
			err = coded(codeUnknownCommand, "cmd", cmd)
		}
		s.logCmd(req, err)
		_ = c.send(errorReply(err))
		return
	}
	// The stream handlers sent their own replies (or an error before the first one).
	s.logCmd(req, err)
}

// knownPodCommands are real firmware commands the demo leaves out: they get "not available in
// the demo simulator" rather than "unknown command", so an agent learns it is the demo's limit.
var knownPodCommands = map[string]bool{
	"dap_start": true, "load_bin": true, "replay": true, "ota_begin": true, "ota_data": true,
	"ota_end": true, "fpga_image": true, "dac_control_loop": true, "dac_loop_input": true,
	"dac_loop_probe": true, "dac_limits": true, "current_out": true, "calibrate": true,
	"calibration": true, "can_start": true, "can_write": true, "can_read": true,
	"can_status": true, "can_term": true, "can_respond": true, "can_stop": true,
	"spi_start": true,
	"spi_stop":  true, "spi_xfer": true, "spi_stream": true, "spi_flash": true, "usb_network": true,
	"lan_tls": true, "sig_policy": true, "lan_policy": true, "wifi_status": true, "dac_set": true,
	"measure": true, "reboot": true, "cloud_config": true, "flash_self": true,
}
