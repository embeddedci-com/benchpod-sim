package benchsim

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"math"
	"net"
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
	"ping":          func(*Sim, request) (any, error) { return "pong", nil },
	"status":        (*Sim).cmdStatus,
	"la_voltage":    (*Sim).cmdLaVoltage,
	"cloud_status":  func(*Sim, request) (any, error) { return map[string]any{"configured": false, "state": "off"}, nil },
	"lease_state":   func(*Sim, request) (any, error) { return map[string]any{"held": false}, nil },
	"target_power":  (*Sim).cmdTargetPower,
	"target_status": (*Sim).cmdTargetStatus,
	"power_status":  (*Sim).cmdPowerStatus,
	"nrst":          (*Sim).cmdNrst,
	"usb_cc":        (*Sim).cmdUsbCc,
	"la":            (*Sim).cmdLa,
	"la_pins":       (*Sim).cmdLaPins,
	"gpio":          (*Sim).cmdGpio,
	"sensor_start":  (*Sim).cmdSensorStart,
	"sensor_set":    (*Sim).cmdSensorSet,
	"sensor_stop":   (*Sim).cmdSensorStop,
	"sensor_status": (*Sim).cmdSensorStatus,
	"sensor_types":  (*Sim).cmdSensorTypes,
	"analog_path":   (*Sim).cmdAnalogPath,
	"dac_out":       (*Sim).cmdDacOut,
	"adc_read":      (*Sim).cmdAdcRead,
	"generate":      (*Sim).cmdGenerate,
	"dac_stop":      (*Sim).cmdDacStop,
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
		_ = c.send(errReply(err.Error()))
		return
	}
	if h, ok := simpleHandlers[cmd]; ok {
		data, err := h(s, req)
		s.logCmd(req, err)
		if err != nil {
			_ = c.send(errReply(err.Error()))
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
	case "sensor_regs":
		err = s.cmdSensorRegs(c, req)
	case "capture":
		err = s.cmdCapture(c, req)
	case "capture_dual":
		err = s.cmdCaptureDual(c, req)
	case "power_profile":
		err = s.cmdPowerProfile(c, req)
	default:
		err = notInDemo(cmd)
		if !knownPodCommands[cmd] {
			err = errf("unknown command: %s", cmd)
		}
		s.logCmd(req, err)
		_ = c.send(errReply(err.Error()))
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
	"calibration": true, "can_config": true, "can_write": true, "can_read": true,
	"can_status": true, "can_term": true, "can_respond": true, "can_disable": true,
	"gps_start": true, "gps_stop": true, "gps_set": true, "gps_status": true, "spi_start": true,
	"spi_stop": true, "spi_xfer": true, "spi_stream": true, "spi_flash": true, "usb_net": true,
	"lan_tls": true, "sig_policy": true, "lan_policy": true, "wifi": true, "dac_set": true,
	"measure": true, "psram_recover": true, "cloud_config": true, "flash_self": true,
}
