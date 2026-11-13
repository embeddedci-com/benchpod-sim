package benchsim

import (
	"encoding/base64"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"math"
	"regexp"
	"slices"
	"sort"
	"strconv"
	"strings"

	"github.com/embeddedci-com/benchpod-sim/sensorengine"
)

// The pod's emulated sensors from definitions (firmware 3.10.3 and later, API.md "Emulated
// sensors"): the host uploads a definition, the pod compiles it and plays it on LA channels or
// the analog outputs. The simulator compiles and runs definitions with the pod's own engine
// (sensorengine, the firmware's C as WASM), so the register images, UART messages and analog
// values are what a real pod would put out. What the gateware does around them (the I2C
// target, conversion times, faults on the bus) is modeled here.

const (
	maxDefined     = 8
	maxI2CSensors  = 4
	maxUploadBytes = 32768
	maxChunkB64    = 1240
	maxFaults      = 4
	uploadIdleNs   = int64(60e9)
	maxConvUs      = 65535
	// imageRefreshNs is how often an image with dynamic registers is rebuilt while nothing
	// changes, for definitions whose readings move with time.
	imageRefreshNs = int64(1e9)
)

var instanceRe = regexp.MustCompile(`^[a-z0-9_]{1,16}$`)

// sensorParam is one param of a compiled definition.
type sensorParam struct {
	Key   string  `json:"key"`
	Unit  string  `json:"unit"`
	Label string  `json:"label"`
	Min   float64 `json:"min"`
	Max   float64 `json:"max"`
	Def   float64 `json:"default"`
}

type busCommand struct {
	At      int `json:"at"`
	BusyUs  int `json:"busy_us"`
	Command int `json:"command"`
}

type mapEntry struct {
	Addr int `json:"addr"`
	At   int `json:"at"`
	Span int `json:"span"`
}

// defReport is the part of the engine's validate answer the simulator uses.
type defReport struct {
	Kind         string        `json:"kind"`
	Name         string        `json:"name"`
	Label        string        `json:"label"`
	ProgramBytes int           `json:"program_bytes"`
	Params       []sensorParam `json:"params"`
	Bus          *struct {
		Address       int        `json:"address"`
		Alt           []int      `json:"alt"`
		AddrBits      int        `json:"addr_bits"`
		Page          int        `json:"page"`
		AutoIncrement bool       `json:"auto_increment"`
		Wrap          int        `json:"wrap"`
		RegisterBytes int        `json:"register_bytes"`
		CommandMap    bool       `json:"command_map"`
		Map           []mapEntry `json:"map"`
		SPIMode       *int       `json:"spi_mode"`
		Conversion    *struct {
			Trigger   int    `json:"trigger"`
			BusyAt    int    `json:"busy_at"`
			BusyMask  int    `json:"busy_mask"`
			WhileBusy string `json:"while_busy"`
			Us        int    `json:"us"`
		} `json:"conversion"`
		Commands []busCommand `json:"commands"`
	} `json:"bus"`
	Dynamic  []int `json:"dynamic"`
	ReadOnly []int `json:"read_only"`
	UART     *struct {
		Baud     int    `json:"baud"`
		DataBits int    `json:"data_bits"`
		Parity   string `json:"parity"`
		StopBits int    `json:"stop_bits"`
	} `json:"uart"`
	Analog *struct {
		Output     string  `json:"output"`
		Excitation bool    `json:"excitation"`
		Min        float64 `json:"min"`
		Max        float64 `json:"max"`
	} `json:"analog"`
}

func (d *defReport) param(key string) *sensorParam {
	for i := range d.Params {
		if d.Params[i].Key == key {
			return &d.Params[i]
		}
	}
	return nil
}

func (d *defReport) uartFormat() string {
	if d.UART == nil {
		return ""
	}
	return fmt.Sprintf("%d%s%d", d.UART.DataBits, d.UART.Parity, d.UART.StopBits)
}

// sensorFault is one fault switched on for an instance.
type sensorFault struct {
	typ     string
	param   string
	value   *float64
	at      int
	mask    int
	hasAt   bool
	rateHz  float64
	until   int64 // 0: until cleared
	stim    bool  // the stimulus switched it on
	stimIdx int
	frozen  map[string]float64 // stuck: the readings when it started
}

// sensorInst is one instance: its definition and, while it runs, its pins and state.
type sensorInst struct {
	name    string
	def     []byte
	rep     *defReport
	running bool
	slot    string
	pins    map[string]int
	output  string // analog: dac or current
	addr    int
	baud    int
	values  map[string]float64
	startAt int64
	faults  []*sensorFault

	// I2C/SPI: the register image the DUT reads, and what makes it stale.
	img       [256]byte
	dirty     bool
	builtAt   int64
	ptr       int
	busyUntil int64
	busyNack  bool
	writes    int
	lastAt    int
	lastVal   int
	wrote     bool

	// UART: the next message epoch (ms since start), and the counters.
	nextMs                       float64
	sentBytes, messages, dropped int
	rng                          uint32

	// analog: the value last driven.
	outValue float64
}

func (e *sensorInst) kind() string { return e.rep.Kind }

// sensorUpload is the one definition or stimulus upload in flight.
type sensorUpload struct {
	stimulus bool
	instance string
	total    int
	data     []byte
	at       int64
}

// stimState is the loaded stimulus.
type stimState struct {
	def     []byte
	rep     stimReport
	running bool
	t0      int64
	last    int64
	applied bool
}

type stimFault struct {
	Type      string   `json:"type"`
	AtS       float64  `json:"at_s"`
	DurationS *float64 `json:"duration_s"`
	Sensor    string   `json:"sensor"`
	Param     string   `json:"param"`
	Value     *float64 `json:"value"`
	RateHz    float64  `json:"rate_hz"`
	At        *int     `json:"at"`
	Mask      *int     `json:"mask"`
}

type stimReport struct {
	ProgramBytes int     `json:"program_bytes"`
	RateHz       float64 `json:"rate_hz"`
	DurationS    float64 `json:"duration_s"`
	Repeat       bool    `json:"repeat"`
	Signals      []struct {
		Name     string `json:"name"`
		Instance string `json:"instance"`
		Param    string `json:"param"`
	} `json:"signals"`
	Faults []stimFault `json:"faults"`
}

// podSensors is everything the pod holds for the emulated sensors.
type podSensors struct {
	inst   map[string]*sensorInst
	order  []string // definition order, what sensor_status lists
	upload *sensorUpload
	stim   *stimState
}

func engine() (*sensorengine.Engine, error) {
	e, err := sensorengine.Shared()
	if err != nil {
		return nil, fmt.Errorf("sensor engine: %w", err)
	}
	return e, nil
}

func hex2(v int) string { return fmt.Sprintf("0x%02X", v) }

// fmtNum prints a number the way the firmware's replies do: integers without a fraction.
func fmtNum(v float64) string { return strconv.FormatFloat(v, 'g', -1, 64) }

// laArg reads a pin value: 3 or "LA3".
func laArg(v any) (int, bool) {
	switch x := v.(type) {
	case float64:
		if x == math.Trunc(x) && x >= 1 && x <= 14 {
			return int(x), true
		}
	case string:
		s := strings.TrimPrefix(strings.ToUpper(strings.TrimSpace(x)), "LA")
		if n, err := strconv.Atoi(s); err == nil && n >= 1 && n <= 14 {
			return n, true
		}
	}
	return 0, false
}

// intArg reads an integer or a hex string ("0x77").
func intArg(v any) (int, bool) {
	r := request{"v": v}
	f, ok := r.num("v")
	if !ok || f != math.Trunc(f) {
		return 0, false
	}
	return int(f), true
}

func (s *Sim) sensorInstance(r request) (*sensorInst, error) {
	name := r.str("instance")
	e := s.pod.sensors.inst[name]
	if e == nil {
		return nil, errf("unknown sensor instance '%s'", name)
	}
	return e, nil
}

// -- uploads ---------------------------------------------------------------------------------

// uploadChunk takes one chunk of a sensor_define or sensor_stimulus upload. It returns the whole
// upload when this chunk completes it.
func (s *Sim) uploadChunk(r request, stimulus bool, instance string) ([]byte, int, error) {
	p := &s.pod.sensors
	now := s.now()
	if p.upload != nil && now-p.upload.at > uploadIdleNs {
		p.upload = nil
	}
	for _, k := range []string{"offset", "total", "data_b64"} {
		if !r.has(k) {
			return nil, 0, errf("missing %s", k)
		}
	}
	off, total := r.int("offset", -1), r.int("total", 0)
	if total < 1 || total > maxUploadBytes {
		return nil, 0, fwError("total must be 1..32768 bytes")
	}
	b64 := r.str("data_b64")
	if len(b64) > maxChunkB64 {
		return nil, 0, fwError("data_b64 too long: at most 1240 characters per chunk")
	}
	chunk, err := decodeB64(b64)
	if err != nil {
		return nil, 0, fwError("data_b64 is not base64")
	}
	if off == 0 {
		p.upload = &sensorUpload{stimulus: stimulus, instance: instance, total: total}
	}
	u := p.upload
	expected := 0
	if u != nil {
		expected = len(u.data)
	}
	if u == nil || off != len(u.data) || u.stimulus != stimulus || u.instance != instance || u.total != total {
		return nil, 0, errf("chunk at offset %d, expected %d (start again at offset 0)", off, expected)
	}
	if len(u.data)+len(chunk) > total {
		p.upload = nil
		return nil, 0, errf("chunk at offset %d runs past total %d (start again at offset 0)", off, total)
	}
	u.data = append(u.data, chunk...)
	u.at = now
	if len(u.data) < total {
		return nil, len(u.data), nil
	}
	p.upload = nil
	return u.data, len(u.data), nil
}

func decodeB64(s string) ([]byte, error) {
	s = strings.TrimRight(s, "=")
	if strings.ContainsAny(s, "-_") {
		return base64.RawURLEncoding.DecodeString(s)
	}
	return base64.RawStdEncoding.DecodeString(s)
}

// defineReply is the last chunk's reply: what the pod compiled.
func defineReply(name string, rep *defReport) map[string]any {
	out := map[string]any{"instance": name, "kind": rep.Kind, "name": rep.Name, "label": rep.Label,
		"program_bytes": rep.ProgramBytes}
	switch rep.Kind {
	case "i2c":
		alt := []string{}
		for _, a := range rep.Bus.Alt {
			alt = append(alt, hex2(a))
		}
		out["i2c"] = map[string]any{"address": hex2(rep.Bus.Address), "alt_addresses": alt}
	case "spi":
		mode := 0
		if rep.Bus != nil && rep.Bus.SPIMode != nil {
			mode = *rep.Bus.SPIMode
		}
		out["spi"] = map[string]any{"mode": mode}
	case "uart":
		out["uart"] = map[string]any{"baud": rep.UART.Baud, "format": rep.uartFormat()}
	case "analog":
		out["analog"] = map[string]any{"output": rep.Analog.Output, "min": rep.Analog.Min, "max": rep.Analog.Max}
	}
	params := []any{}
	for _, p := range rep.Params {
		params = append(params, map[string]any{"key": p.Key, "unit": p.Unit, "min": p.Min, "max": p.Max, "default": p.Def})
	}
	out["params"] = params
	return out
}

func (s *Sim) cmdSensorDefine(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	name := r.str("instance")
	if !instanceRe.MatchString(name) {
		return nil, fwError("instance must be 1..16 characters of a-z, 0-9 and _")
	}
	data, got, err := s.uploadChunk(r, false, name)
	if err != nil {
		return nil, err
	}
	if data == nil {
		return map[string]any{"instance": name, "received": got}, nil
	}
	eng, err := engine()
	if err != nil {
		return nil, err
	}
	raw, err := eng.Validate(data)
	var ee *sensorengine.Error
	if errors.As(err, &ee) {
		return nil, errf("invalid definition: %s", ee.Error())
	} else if err != nil {
		return nil, err
	}
	rep := &defReport{}
	if err := json.Unmarshal(raw, rep); err != nil {
		return nil, errf("invalid definition: %v", err)
	}
	if rep.Kind == "i2c" && rep.Bus == nil || rep.Kind == "uart" && rep.UART == nil || rep.Kind == "analog" && rep.Analog == nil {
		return nil, errf("invalid definition: the engine reports no %s settings", rep.Kind)
	}
	reply := defineReply(name, rep)
	if r.flag("dry_run") {
		return reply, nil
	}
	p := &s.pod.sensors
	if p.inst[name] == nil && len(p.inst) >= maxDefined {
		return nil, fwError("8 instances are defined already: sensor_stop one first")
	}
	now := s.now()
	s.advance(now)
	if old := p.inst[name]; old != nil {
		s.sensorHalt(now, old)
	} else {
		p.order = append(p.order, name)
	}
	if p.inst == nil {
		p.inst = map[string]*sensorInst{}
	}
	p.inst[name] = &sensorInst{name: name, def: data, rep: rep, values: defaults(rep)}
	return reply, nil
}

func defaults(rep *defReport) map[string]float64 {
	v := map[string]float64{}
	for _, p := range rep.Params {
		v[p.Key] = p.Def
	}
	return v
}

// checkValues reads a values object and checks every value before any is applied.
func checkValues(e *sensorInst, raw any) (map[string]float64, error) {
	obj, ok := raw.(map[string]any)
	if !ok {
		return nil, fwError("values must be an object")
	}
	out := map[string]float64{}
	keys := make([]string, 0, len(obj))
	for k := range obj {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	for _, k := range keys {
		p := e.rep.param(k)
		if p == nil {
			return nil, errf("%s has no param '%s'", e.name, k)
		}
		v, ok := (request{"v": obj[k]}).num("v")
		if _, isBool := obj[k].(bool); !ok || isBool || v < p.Min || v > p.Max {
			return nil, errf("%s must be %s..%s %s", k, fmtNum(p.Min), fmtNum(p.Max), p.Unit)
		}
		out[k] = v
	}
	return out, nil
}

// -- start, set, stop ------------------------------------------------------------------------

// pinRoles are the pins each kind takes, required ones first, with the LA function they claim.
var pinRoles = map[string][]struct {
	role, fn string
	need     bool
}{
	"i2c":  {{"sda", "i2c_sda", true}, {"scl", "i2c_scl", true}, {"drdy", "sensor_drdy", false}},
	"spi":  {{"sclk", "sensor_sck", true}, {"mosi", "sensor_mosi", true}, {"miso", "sensor_miso", true}, {"cs", "sensor_cs", true}},
	"uart": {{"tx", "sensor_tx", true}, {"rx", "sensor_rx", false}},
}

var pinsNeeded = map[string]string{
	"i2c":  "an I2C sensor needs pins sda and scl",
	"spi":  "an SPI sensor needs pins sclk, mosi, miso and cs",
	"uart": "a UART sensor needs pin tx",
}

func imageName(img int) string { return []string{"loop", "deep", "emulation"}[img] }

func (s *Sim) dacWaveformPlaying(now int64) bool {
	d := s.pod.analog.dac
	switch d.kind {
	case dacWave:
		return d.until == 0 || now < d.until
	case dacReplay, dacLoopKind:
		return true
	}
	return false
}

// analogOn is the analog instance playing an output (dac or current), if any.
func (s *Sim) analogOn(output string) *sensorInst {
	for _, n := range s.pod.sensors.order {
		if e := s.pod.sensors.inst[n]; e != nil && e.running && e.kind() == "analog" && e.output == output {
			return e
		}
	}
	return nil
}

// dacDrivers are the commands that drive the DAC, refused while an analog sensor plays it.
var dacDrivers = map[string]string{
	"generate": "dac", "replay": "dac", "load": "dac", "load_bin": "dac", "measure": "dac",
	"dac_set": "dac", "dac_out": "dac", "dac_control_loop": "dac", "current_out": "current",
}

// analogGuard refuses a DAC command while an analog sensor plays the output it drives.
func (s *Sim) analogGuard(cmd string) error {
	out, ok := dacDrivers[cmd]
	if !ok {
		return nil
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	if e := s.analogOn(out); e != nil {
		return errf(`the %s output is played by sensor:%s; stop it first ({"cmd":"sensor_stop","instance":"%s"})`, out, e.name, e.name)
	}
	return nil
}

func (s *Sim) cmdSensorStart(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e, err := s.sensorInstance(r)
	if err != nil {
		return nil, err
	}
	if e.running {
		return nil, errf("%s is running: sensor_stop it first", e.name)
	}
	kind := e.kind()
	pinsObj, _ := r["pins"].(map[string]any)
	if pinsObj == nil {
		if kind == "analog" {
			return nil, fwError(`an analog sensor needs pins {"output":"dac"} or {"output":"current"}`)
		}
		return nil, fwError(pinsNeeded[kind])
	}
	now := s.now()
	s.advance(now)
	p := &s.pod.sensors
	reply := map[string]any{"instance": e.name}
	pins := map[string]int{}
	addr, baud, output, slot := 0, 0, "", ""
	switch kind {
	case "analog":
		output = strings.ToLower(fmt.Sprint(pinsObj["output"]))
		if output != "dac" && output != "current" {
			return nil, fwError(`an analog sensor's output is "dac" or "current"`)
		}
		if e.rep.Analog.Excitation {
			return nil, fwError("excitation: adc is not supported by the pod yet")
		}
		if o := s.analogOn(output); o != nil {
			return nil, errf(`the %s output is played by sensor:%s; stop it first ({"cmd":"sensor_stop","instance":"%s"})`, output, o.name, o.name)
		}
		if output == "dac" && s.dacWaveformPlaying(now) {
			return nil, fwError("a DAC waveform is playing: dac_stop it first")
		}
		slot = map[string]string{"dac": "analog0", "current": "analog1"}[output]
	default:
		if err := s.requireLaVoltage(); err != nil {
			return nil, err
		}
		if kind == "spi" && pinsObj["drdy"] != nil {
			return nil, fwError("the emulated SPI target has no data-ready pin")
		}
		for _, pr := range pinRoles[kind] {
			v, ok := pinsObj[pr.role]
			if !ok || v == nil {
				if pr.need {
					return nil, fwError(pinsNeeded[kind])
				}
				continue
			}
			la, ok := laArg(v)
			if !ok {
				return nil, errf("pin %s must be an LA channel 1..14", pr.role)
			}
			for role, other := range pins {
				if other == la {
					return nil, errf("pins %s and %s are both LA%d", role, pr.role, la)
				}
			}
			pins[pr.role] = la
		}
		if err := s.sensorClaimCheck(kind, pins); err != nil {
			return nil, err
		}
		switch kind {
		case "i2c":
			addr = e.rep.Bus.Address
			if r.has("address") {
				a, ok := intArg(r["address"])
				if !ok || a != e.rep.Bus.Address && !slices.Contains(e.rep.Bus.Alt, a) {
					return nil, errf("address %s is not one of this sensor's addresses", r.str("address"))
				}
				addr = a
			}
			used := map[string]bool{}
			for _, n := range p.order {
				o := p.inst[n]
				if o == nil || !o.running || o.kind() != "i2c" {
					continue
				}
				used[o.slot] = true
				if o.pins["sda"] == pins["sda"] && o.addr == addr {
					return nil, errf("address %s is used by sensor:%s on this bus", hex2(addr), o.name)
				}
			}
			for i := 0; i < maxI2CSensors; i++ {
				if c := fmt.Sprintf("i2c%d", i); !used[c] {
					slot = c
					break
				}
			}
			if slot == "" {
				return nil, fwError("all 4 I2C sensor slots are in use: sensor_stop one first")
			}
			reply["address"] = hex2(addr)
		case "spi", "uart":
			slot = kind + "0"
			for _, n := range p.order {
				if o := p.inst[n]; o != nil && o.running && o.kind() == kind {
					return nil, errf("the %s sensor slot is used by sensor:%s: sensor_stop it first", strings.ToUpper(kind), o.name)
				}
			}
			if kind == "uart" {
				if f := e.rep.uartFormat(); f != "8N1" {
					return nil, errf("the emulated UART is 8N1 only (the definition asks for %s)", f)
				}
				baud = e.rep.UART.Baud
				if r.has("baud") {
					baud = r.int("baud", 0)
					if baud < 300 || baud > 3000000 {
						return nil, fwError("baud must be 300..3000000")
					}
				}
				reply["baud"] = baud
			}
		}
	}
	vals := defaults(e.rep)
	if r.has("values") {
		v, err := checkValues(e, r["values"])
		if err != nil {
			return nil, err
		}
		for k, x := range v {
			vals[k] = x
		}
	}
	loaded := false
	if kind != "analog" && s.pod.image != 2 {
		if s.dacWaveformPlaying(now) {
			return nil, fwError("loading the emulation image stops the DAC waveform that is playing: dac_stop it first")
		}
		s.switchImage(now, 2)
		loaded = true
	}
	for _, pr := range pinRoles[kind] {
		if la, ok := pins[pr.role]; ok {
			s.pod.pins[la] = laPin{fn: pr.fn}
		}
	}
	*e = sensorInst{name: e.name, def: e.def, rep: e.rep, running: true, slot: slot, pins: pins, output: output,
		addr: addr, baud: baud, values: vals, startAt: now, dirty: true, rng: 0x2545F491}
	switch kind {
	case "i2c", "spi":
		s.rebuildImage(e, now, true)
	case "uart":
		e.nextMs = 0
		s.uartAdvance(e, now)
	case "analog":
		s.analogDrive(e, now)
	}
	s.touchBase(now)
	reply["slot"] = slot
	reply["image"] = imageName(s.pod.image)
	if kind != "analog" {
		reply["image_loaded"] = loaded
	}
	if s.pod.image == 2 {
		reply["unavailable"] = []string{"dac_control_loop", "deep_replay"}
	}
	return reply, nil
}

// sensorClaimCheck checks the pins an instance claims: free, or (I2C) the same pair other I2C
// sensors use; pull-downs under lines that idle high are refused.
func (s *Sim) sensorClaimCheck(kind string, pins map[string]int) error {
	for _, pr := range pinRoles[kind] {
		la, ok := pins[pr.role]
		if !ok {
			continue
		}
		cur := s.pod.pins[la]
		if cur.fn != "none" {
			shared := false
			if kind == "i2c" && cur.fn == pr.fn && (pr.fn == "i2c_sda" || pr.fn == "i2c_scl") {
				// Another I2C sensor's line: it may be shared when the whole pair is the same.
				for _, o := range s.sensorsOn(la) {
					shared = o.pins["sda"] == pins["sda"] && o.pins["scl"] == pins["scl"]
				}
			}
			if !shared {
				return errf("pin conflict: LA%d is in use by %s; %s", la, s.pinOwnerText(la), s.releaseHint(la))
			}
		}
		if pullDown(la) && s.pod.pulls&(1<<(la-1)) != 0 {
			switch pr.fn {
			case "i2c_sda", "i2c_scl":
				return errf("pull conflict: LA%d has its 10k pull-down engaged, which %s can't work with (an open-drain bus needs pull-ups); disable it first", la, pr.fn)
			case "sensor_tx", "sensor_rx":
				return errf("pull conflict: LA%d has its 10k pull-down engaged, which %s can't work with (the line idles high); disable it first", la, pr.fn)
			}
		}
	}
	return nil
}

// sensorsOn lists the running instances that hold an LA channel.
func (s *Sim) sensorsOn(la int) []*sensorInst {
	var out []*sensorInst
	for _, n := range s.pod.sensors.order {
		e := s.pod.sensors.inst[n]
		if e == nil || !e.running {
			continue
		}
		for _, v := range e.pins {
			if v == la {
				out = append(out, e)
				break
			}
		}
	}
	return out
}

// pinOwner is la_pins' "owner" of a sensor pin: "sensor:env", or "sensor:env+1" on a shared line.
func (s *Sim) pinOwner(la int) string {
	on := s.sensorsOn(la)
	switch len(on) {
	case 0:
		return ""
	case 1:
		return "sensor:" + on[0].name
	}
	return fmt.Sprintf("sensor:%s+%d", on[0].name, len(on)-1)
}

// pinOwnerText names a pin's owner in a conflict: the function, or the sensor and its function.
func (s *Sim) pinOwnerText(la int) string {
	fn := s.pod.pins[la].fn
	if on := s.sensorsOn(la); len(on) > 0 {
		return fmt.Sprintf("sensor:%s (%s)", on[0].name, fn)
	}
	return fn
}

func (s *Sim) cmdSensorSet(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e, err := s.sensorInstance(r)
	if err != nil {
		return nil, err
	}
	if !r.has("values") {
		return nil, fwError("missing values")
	}
	vals, err := checkValues(e, r["values"])
	if err != nil {
		return nil, err
	}
	now := s.now()
	s.advance(now)
	for k, v := range vals {
		e.values[k] = v
	}
	s.sensorChanged(e, now)
	return map[string]any{"instance": e.name, "values": r["values"]}, nil
}

// sensorChanged applies new readings: a stale image, a new analog level.
func (s *Sim) sensorChanged(e *sensorInst, now int64) {
	e.dirty = true
	if e.running && e.kind() == "analog" {
		s.analogDrive(e, now)
	}
}

// sensorHalt stops an instance: its pins, its output, a stimulus that drives it. It stays defined.
func (s *Sim) sensorHalt(now int64, e *sensorInst) {
	if !e.running {
		return
	}
	if st := s.pod.sensors.stim; st != nil && st.running && s.stimDrives(st, e) {
		s.stimStop()
	}
	e.running = false
	e.faults = nil
	for _, la := range e.pins {
		if len(s.sensorsOn(la)) == 0 {
			s.pod.pins[la] = laPin{fn: "none"}
		}
	}
	if e.kind() == "analog" && e.output == "dac" {
		s.snapshotAnalog(now)
		d := &s.pod.analog.dac
		*d = dacState{path: d.path}
	}
	s.touchBase(now)
}

func (s *Sim) cmdSensorStop(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	p := &s.pod.sensors
	var names []string
	if r.flag("all") {
		names = append(names, p.order...)
	} else {
		e, err := s.sensorInstance(r)
		if err != nil {
			return nil, err
		}
		names = []string{e.name}
	}
	now := s.now()
	s.advance(now)
	stopped := []string{}
	for _, n := range names {
		if e := p.inst[n]; e != nil {
			s.sensorHalt(now, e)
			delete(p.inst, n)
			p.order = slices.DeleteFunc(p.order, func(x string) bool { return x == n })
			stopped = append(stopped, n)
		}
	}
	return map[string]any{"stopped": stopped}, nil
}

// sensorsDacStop is dac_stop's part: it stops the analog instances on the DAC (they stay defined).
func (s *Sim) sensorsDacStop(now int64) {
	for _, n := range s.pod.sensors.order {
		if e := s.pod.sensors.inst[n]; e != nil && e.running && e.kind() == "analog" && e.output == "dac" {
			s.sensorHalt(now, e)
		}
	}
}

// -- status and registers ----------------------------------------------------------------------

func (s *Sim) cmdSensorStatus(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	p := &s.pod.sensors
	names := p.order
	if r.has("instance") {
		e, err := s.sensorInstance(r)
		if err != nil {
			return nil, err
		}
		names = []string{e.name}
	}
	first := max(r.int("first", 0), 0)
	list := []any{}
	for i, n := range names {
		if i < first {
			continue
		}
		if e := p.inst[n]; e != nil {
			list = append(list, s.instanceStatus(e, now))
		}
	}
	stim := map[string]any{"loaded": p.stim != nil, "running": false, "t_s": nil}
	if st := p.stim; st != nil && st.running {
		stim["running"] = true
		stim["t_s"] = math.Round(s.stimTime(st, now)*100) / 100
	}
	return map[string]any{"image": imageName(s.pod.image), "stimulus": stim, "instances": list}, nil
}

func (s *Sim) instanceStatus(e *sensorInst, now int64) map[string]any {
	out := map[string]any{"instance": e.name, "kind": e.kind(), "name": e.rep.Name, "slot": "", "running": e.running}
	vals := map[string]any{}
	for _, p := range e.rep.Params {
		vals[p.Key] = e.values[p.Key]
	}
	out["values"] = vals
	faults := []any{}
	for _, f := range s.activeFaults(e, now) {
		fo := map[string]any{"type": f.typ, "left_s": nil}
		if f.param != "" {
			fo["param"] = f.param
		}
		if f.value != nil {
			fo["value"] = *f.value
		}
		if f.hasAt {
			fo["at"], fo["mask"] = hex2(f.at), hex2(f.mask)
		}
		if f.typ == "garbage" {
			fo["rate_hz"] = f.rateHz
		}
		if f.stim {
			fo["stimulus"] = true
		}
		if f.until > 0 {
			fo["left_s"] = math.Round(float64(f.until-now)/1e7) / 100
		}
		faults = append(faults, fo)
	}
	out["faults"] = faults
	if !e.running {
		out["pins"] = map[string]any{}
		return out
	}
	out["slot"] = e.slot
	pins := map[string]any{}
	for k, v := range e.pins {
		pins[k] = v
	}
	if e.kind() == "analog" {
		pins["output"] = e.output
	}
	out["pins"] = pins
	switch e.kind() {
	case "i2c", "spi":
		if e.kind() == "i2c" {
			out["address"] = hex2(e.addr)
		}
		var last any
		if e.wrote {
			last = map[string]any{"at": hex2(e.lastAt), "value": hex2(e.lastVal)}
		}
		out["bus"] = map[string]any{"writes": e.writes, "transfers": nil, "log_lost": 0, "last_write": last,
			"drdy": 0, "fifo_frames": 0, "fifo_drops": 0}
	case "uart":
		out["baud"] = e.baud
		out["bus"] = map[string]any{"sent_bytes": e.sentBytes, "received_bytes": 0, "messages": e.messages,
			"answers": 0, "overruns": 0, "dropped": e.dropped, "late": 0}
	case "analog":
		out["output"] = math.Round(e.outValue*1e6) / 1e6
	}
	return out
}

func (s *Sim) cmdSensorRegs(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e, err := s.sensorInstance(r)
	if err != nil {
		return nil, err
	}
	if k := e.kind(); k != "i2c" && k != "spi" {
		return nil, errf("sensor_regs reads I2C and SPI sensors only (%s is a %s sensor)", e.name, k)
	}
	if !e.running {
		return nil, errf("%s is not running: sensor_start it first", e.name)
	}
	start, n := 0, 0
	if r.has("start") {
		v, ok := intArg(r["start"])
		if !ok || v < 0 || v > 255 {
			return nil, fwError("start must be 0..255")
		}
		start = v
	}
	n = 256 - start
	if r.has("len") {
		v, ok := intArg(r["len"])
		if !ok || v < 1 || v > 256 {
			return nil, fwError("len must be 1..256")
		}
		n = v
	}
	if start+n > 256 {
		return nil, errf("start 0x%02X + len %d runs past the 256-byte image", start, n)
	}
	now := s.now()
	s.advance(now)
	s.refreshImage(e, now)
	data := make([]byte, n)
	for i := range data {
		data[i] = s.imageByte(e, now, start+i)
	}
	return map[string]any{"instance": e.name, "start": hex2(start), "len": n,
		"data_b64": base64.StdEncoding.EncodeToString(data)}, nil
}

// -- the register image -------------------------------------------------------------------------

// effective is what an instance reports now: its values with the stimulus and faults applied.
func (s *Sim) effective(e *sensorInst, now int64) map[string]float64 {
	out := map[string]float64{}
	for k, v := range e.values {
		out[k] = v
	}
	for _, f := range s.activeFaults(e, now) {
		switch f.typ {
		case "stuck":
			for k, v := range f.frozen {
				if f.param == "" || f.param == k {
					out[k] = v
				}
			}
		case "offset", "gain":
			for _, p := range e.rep.Params {
				if f.param != "" && f.param != p.Key || f.value == nil {
					continue
				}
				v := out[p.Key] + *f.value
				if f.typ == "gain" {
					v = out[p.Key] * *f.value
				}
				out[p.Key] = math.Max(p.Min, math.Min(p.Max, v))
			}
		}
	}
	return out
}

// rebuildImage runs the engine's preview: from power on, or from the image the DUT wrote.
func (s *Sim) rebuildImage(e *sensorInst, now int64, powerOn bool) {
	eng, err := engine()
	if err != nil {
		log.Printf("benchsim: %v", err)
		return
	}
	vals, _ := json.Marshal(s.effective(e, now))
	var live []byte
	if !powerOn {
		live = e.img[:]
	}
	tMs := float64(now-e.startAt) / 1e6
	raw, err := eng.Preview(e.def, vals, live, tMs, 0)
	if err != nil {
		log.Printf("benchsim: sensor %s: preview: %v", e.name, err)
		return
	}
	var pv struct {
		Image string `json:"image_b64"`
	}
	if json.Unmarshal(raw, &pv) != nil {
		return
	}
	img, err := base64.StdEncoding.DecodeString(pv.Image)
	if err != nil || len(img) != 256 {
		return
	}
	copy(e.img[:], img)
	e.dirty, e.builtAt = false, now
}

// refreshImage rebuilds a stale image before the DUT reads it: new readings, a stimulus step, a
// conversion that ended, or time for definitions whose registers move with it. A stuck fault
// freezes the image.
func (s *Sim) refreshImage(e *sensorInst, now int64) {
	s.stimApply(now)
	if s.hasFault(e, now, "stuck") {
		return
	}
	if e.dirty || len(e.rep.Dynamic) > 0 && now-e.builtAt >= imageRefreshNs {
		s.rebuildImage(e, now, false)
	}
}

// imageByte is one byte as the DUT reads it: the busy flag of a running conversion and a
// corrupt fault on top of the image.
func (s *Sim) imageByte(e *sensorInst, now int64, at int) byte {
	b := e.img[at&0xFF]
	if c := e.conversion(); c != nil && now < e.busyUntil && at == c.BusyAt {
		b |= byte(c.BusyMask)
	}
	for _, f := range s.activeFaults(e, now) {
		if f.typ == "corrupt" && f.hasAt && f.at&0xFF == at&0xFF {
			b ^= byte(f.mask)
		}
	}
	return b
}

func (e *sensorInst) conversion() *struct {
	Trigger   int    `json:"trigger"`
	BusyAt    int    `json:"busy_at"`
	BusyMask  int    `json:"busy_mask"`
	WhileBusy string `json:"while_busy"`
	Us        int    `json:"us"`
} {
	if e.rep.Bus == nil {
		return nil
	}
	return e.rep.Bus.Conversion
}

// index maps a register pointer to its byte in the 256-byte image (the window, word registers,
// the command map).
func (e *sensorInst) index(ptr int) int {
	b := e.rep.Bus
	switch {
	case b.CommandMap:
		for _, m := range b.Map {
			if ptr >= m.Addr && ptr < m.Addr+m.Span {
				return (m.At + ptr - m.Addr) & 0xFF
			}
		}
		return ptr & 0xFF
	case b.RegisterBytes == 2:
		return 2 * (ptr & 0x7F)
	case b.AddrBits == 16:
		return (ptr - b.Page*256) & 0xFF
	}
	return ptr & 0xFF
}

// i2cAcks says whether the instance acknowledges its address now.
func (s *Sim) i2cAcks(e *sensorInst, now int64) bool {
	if s.hasFault(e, now, "no_response") {
		return false
	}
	return !(e.busyNack && now < e.busyUntil)
}

// i2cWrite is the controller writing to the target: the register pointer, then data bytes.
func (s *Sim) i2cWrite(e *sensorInst, now int64, data []byte) {
	if len(data) == 0 {
		return
	}
	b := e.rep.Bus
	n := 1
	if b.AddrBits == 16 {
		n = 2
	}
	if len(data) < n {
		return
	}
	ptr := int(data[0])
	if n == 2 {
		ptr = int(data[0])<<8 | int(data[1])
	}
	idx := e.index(ptr)
	// A command: its answer layout, and the reads NACK while it runs.
	for _, c := range b.Commands {
		if c.Command == ptr {
			idx = c.At & 0xFF
			if c.BusyUs > 0 {
				e.busyUntil = now + int64(min(c.BusyUs, maxConvUs))*1000
				e.busyNack = true
				e.dirty = true
			}
		}
	}
	e.ptr = idx
	readOnly := e.rep.ReadOnly
	for _, v := range data[n:] {
		at := e.ptr
		e.writes++
		e.wrote, e.lastAt, e.lastVal = true, at, int(v)
		if !slices.Contains(readOnly, at) {
			e.img[at] = v
		}
		e.dirty = true
		if c := e.conversion(); c != nil && at == c.Trigger {
			e.busyUntil = now + int64(min(c.Us, maxConvUs))*1000
			e.busyNack = c.WhileBusy == "nack"
		}
		e.ptr = s.nextPtr(e, at)
	}
}

func (s *Sim) nextPtr(e *sensorInst, at int) int {
	b := e.rep.Bus
	if !b.AutoIncrement {
		return at
	}
	wrap := b.Wrap
	if wrap <= 0 || wrap > 255 {
		wrap = 255
	}
	if at >= wrap {
		return 0
	}
	return at + 1
}

// i2cRead is the controller reading n bytes from the target.
func (s *Sim) i2cRead(e *sensorInst, now int64, n int) []byte {
	s.refreshImage(e, now)
	out := make([]byte, n)
	for i := range out {
		out[i] = s.imageByte(e, now, e.ptr)
		e.ptr = s.nextPtr(e, e.ptr)
	}
	return out
}

// sensorAt is the emulated part answering at addr on the node's sensor bus: the right channels
// and address, nothing holding the bus low, and acknowledging now.
func (s *Sim) sensorAt(addr byte, now int64) *sensorInst {
	if s.busHeldLow(now) {
		return nil
	}
	for _, n := range s.pod.sensors.order {
		e := s.pod.sensors.inst[n]
		if e == nil || !e.running || e.kind() != "i2c" || e.addr != int(addr) ||
			e.pins["sda"] != dutSDA || e.pins["scl"] != dutSCL {
			continue
		}
		if !s.i2cAcks(e, now) {
			return nil
		}
		return e
	}
	return nil
}

// sensorHoldsSDA says whether a bus_stuck fault holds an I2C line low now.
func (s *Sim) sensorHoldsSDA(la int, now int64) bool {
	for _, e := range s.sensorsOn(la) {
		if e.kind() == "i2c" && e.pins["sda"] == la && s.hasFault(e, now, "bus_stuck") {
			return true
		}
	}
	return false
}

// -- UART and analog -------------------------------------------------------------------------

// uartAdvance sends the messages due by now on the instance's tx pin.
func (s *Sim) uartAdvance(e *sensorInst, now int64) {
	eng, err := engine()
	if err != nil {
		return
	}
	nowMs := float64(now-e.startAt) / 1e6
	// Only the capture history matters: skip epochs older than it.
	if floor := float64(now-historyNs-e.startAt) / 1e6; e.nextMs < floor {
		e.nextMs = floor
	}
	for i := 0; i < 200 && e.nextMs <= nowMs; i++ {
		at := e.startAt + int64(e.nextMs*1e6)
		vals, _ := json.Marshal(s.effective(e, at))
		raw, err := eng.Preview(e.def, vals, nil, e.nextMs, 0)
		if err != nil {
			// The engine runs every epoch from the start; past its limit, start the clock again.
			log.Printf("benchsim: sensor %s: preview: %v", e.name, err)
			e.startAt, e.nextMs = now, float64(1)
			return
		}
		var pv struct {
			AtMs   *float64 `json:"at_ms"`
			Bytes  string   `json:"bytes_b64"`
			NextMs *float64 `json:"next_ms"`
		}
		if json.Unmarshal(raw, &pv) != nil {
			return
		}
		if pv.AtMs != nil && *pv.AtMs <= e.nextMs {
			data, _ := base64.StdEncoding.DecodeString(pv.Bytes)
			s.uartEmit(e, e.startAt+int64(*pv.AtMs*1e6), data)
		}
		if pv.NextMs == nil || *pv.NextMs <= e.nextMs {
			e.nextMs = math.Inf(1)
			return
		}
		e.nextMs = *pv.NextMs
	}
}

func (e *sensorInst) rand() uint32 {
	e.rng ^= e.rng << 13
	e.rng ^= e.rng >> 17
	e.rng ^= e.rng << 5
	return e.rng
}

// uartEmit puts one message on the wire, through the UART faults.
func (s *Sim) uartEmit(e *sensorInst, at int64, data []byte) {
	if s.hasFault(e, at, "no_response") {
		return
	}
	if f := s.fault(e, at, "garbage"); f != nil {
		n := len(data)
		data = make([]byte, n)
		for i := range data {
			data[i] = byte(e.rand())
		}
	}
	if f := s.fault(e, at, "drop"); f != nil && f.value != nil && float64(e.rand()%10000)/10000 < *f.value {
		e.dropped++
		return
	}
	if len(data) == 0 {
		return
	}
	byteNs := int64(10e9 / float64(e.baud))
	s.sig.senTx.write(at, e.pins["tx"], byteNs, data)
	e.sentBytes += len(data)
	e.messages++
}

// analogDrive sets the output an analog instance plays to its value now.
func (s *Sim) analogDrive(e *sensorInst, now int64) {
	value := e.outValue
	if !s.hasFault(e, now, "stuck") {
		eng, err := engine()
		if err != nil {
			return
		}
		vals, _ := json.Marshal(s.effective(e, now))
		raw, err := eng.Preview(e.def, vals, nil, float64(now-e.startAt)/1e6, 0)
		if err != nil {
			log.Printf("benchsim: sensor %s: preview: %v", e.name, err)
			return
		}
		var pv struct {
			Value *float64 `json:"value"`
		}
		if json.Unmarshal(raw, &pv) != nil || pv.Value == nil {
			return
		}
		value = *pv.Value
	}
	a := e.rep.Analog
	for _, f := range s.activeFaults(e, now) {
		switch f.typ {
		case "open":
			value = a.Max
			if f.value != nil {
				value = *f.value
			}
		case "short":
			value = a.Min
			if f.value != nil {
				value = *f.value
			}
		}
	}
	e.outValue = value
	if e.output != "dac" {
		return
	}
	path := "12v"
	switch {
	case a.Min >= 0 && a.Max <= 3.3:
		path = "3v3"
	case a.Min >= 0 && a.Max <= 5:
		path = "5v"
	}
	line := s.dacLineFor(pathIndex(path))
	code := max(0, min(math.Round((value-line.a)/line.b*256), 65535))
	if d := &s.pod.analog.dac; d.path != path || d.kind != dacHold || d.code != code {
		s.snapshotAnalog(now)
		*d = dacState{path: path, code: code}
	}
}

// sensorsAdvance runs the instances up to t: faults that end, the stimulus, UART messages,
// analog outputs. Callers hold s.mu.
func (s *Sim) sensorsAdvance(t int64) {
	p := &s.pod.sensors
	if len(p.inst) == 0 {
		return
	}
	s.stimApply(t)
	for _, n := range p.order {
		e := p.inst[n]
		if e == nil || !e.running {
			continue
		}
		held := false
		e.faults = slices.DeleteFunc(e.faults, func(f *sensorFault) bool {
			if f.until > 0 && f.until <= t {
				held = held || f.typ == "bus_stuck"
				return true
			}
			return false
		})
		if held {
			s.touchBase(t)
		}
		switch e.kind() {
		case "uart":
			s.uartAdvance(e, t)
		case "analog":
			if e.dirty {
				s.analogDrive(e, t)
				e.dirty = false
			}
		}
	}
}

// -- faults ----------------------------------------------------------------------------------

var faultKinds = map[string][]string{
	"no_response": {"i2c", "spi", "uart"},
	"bus_stuck":   {"i2c"},
	"stuck":       {"i2c", "spi", "uart", "analog"},
	"offset":      {"i2c", "spi", "uart", "analog"},
	"gain":        {"i2c", "spi", "uart", "analog"},
	"corrupt":     {"i2c", "spi"},
	"garbage":     {"uart"},
	"drop":        {"uart"},
	"baud_error":  {"uart"},
	"open":        {"analog"},
	"short":       {"analog"},
}

func (s *Sim) activeFaults(e *sensorInst, now int64) []*sensorFault {
	var out []*sensorFault
	for _, f := range e.faults {
		if f.until == 0 || now < f.until {
			out = append(out, f)
		}
	}
	return out
}

func (s *Sim) fault(e *sensorInst, now int64, typ string) *sensorFault {
	for _, f := range s.activeFaults(e, now) {
		if f.typ == typ {
			return f
		}
	}
	return nil
}

func (s *Sim) hasFault(e *sensorInst, now int64, typ string) bool { return s.fault(e, now, typ) != nil }

// newFault checks a fault for an instance and builds it.
func newFault(e *sensorInst, typ string, durS *float64, param string, value *float64, at, mask *int, rate float64) (*sensorFault, error) {
	kinds, ok := faultKinds[typ]
	if !ok {
		return nil, errf("unknown fault type '%s'", typ)
	}
	if !slices.Contains(kinds, e.kind()) {
		return nil, errf("fault %s does not apply to a%s %s sensor", typ, map[bool]string{true: "n", false: ""}[e.kind() == "i2c" || e.kind() == "analog"], e.kind())
	}
	if param != "" && e.rep.param(param) == nil {
		return nil, errf("%s has no param '%s'", e.name, param)
	}
	if typ == "bus_stuck" && (durS == nil || *durS > 1) {
		return nil, fwError("bus_stuck needs a duration_s of at most 1 s")
	}
	if (typ == "offset" || typ == "gain" || typ == "drop" || typ == "baud_error") && value == nil {
		return nil, errf("fault %s needs a value", typ)
	}
	f := &sensorFault{typ: typ, param: param, value: value, rateHz: rate}
	if typ == "corrupt" {
		if at == nil || mask == nil {
			return nil, fwError("fault corrupt needs at and mask")
		}
		f.at, f.mask, f.hasAt = *at, *mask&0xFF, true
	}
	if typ == "garbage" && f.rateHz <= 0 {
		f.rateHz = 100
	}
	return f, nil
}

func (s *Sim) addFault(e *sensorInst, now int64, f *sensorFault, durS *float64) {
	if durS != nil {
		f.until = now + int64(*durS*1e9)
	}
	if f.typ == "stuck" {
		f.frozen = s.effective(e, now)
	}
	e.faults = append(e.faults, f)
	if f.typ == "bus_stuck" {
		s.touchBase(now)
	}
	s.sensorChanged(e, now)
}

func (s *Sim) cmdSensorFault(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e, err := s.sensorInstance(r)
	if err != nil {
		return nil, err
	}
	if !e.running {
		return nil, errf("%s is not running: sensor_start it first", e.name)
	}
	num := func(k string) *float64 {
		if v, ok := r.num(k); ok && r.has(k) {
			return &v
		}
		return nil
	}
	intp := func(k string) *int {
		if !r.has(k) {
			return nil
		}
		if v, ok := intArg(r[k]); ok {
			return &v
		}
		return nil
	}
	durS := num("duration_s")
	if durS != nil && *durS <= 0 {
		return nil, fwError("duration_s must be above 0")
	}
	rate, _ := r.num("rate_hz")
	f, err := newFault(e, r.str("type"), durS, r.str("param"), num("value"), intp("at"), intp("mask"), rate)
	if err != nil {
		return nil, err
	}
	now := s.now()
	s.advance(now)
	if len(s.activeFaults(e, now)) >= maxFaults {
		return nil, errf("at most 4 faults are on per instance: sensor_fault_clear %s first", e.name)
	}
	s.addFault(e, now, f, durS)
	return map[string]any{"instance": e.name, "fault": f.typ}, nil
}

func (s *Sim) cmdSensorFaultClear(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	e, err := s.sensorInstance(r)
	if err != nil {
		return nil, err
	}
	now := s.now()
	s.advance(now)
	e.faults = nil
	s.sensorChanged(e, now)
	s.touchBase(now)
	return map[string]any{"instance": e.name, "faults": []any{}}, nil
}

// -- stimulus --------------------------------------------------------------------------------

func signalName(instance, param string) string {
	if instance == "" {
		return param
	}
	return instance + "." + param
}

func (s *Sim) cmdSensorStimulus(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	data, got, err := s.uploadChunk(r, true, "")
	if err != nil {
		return nil, err
	}
	if data == nil {
		return map[string]any{"received": got}, nil
	}
	eng, err := engine()
	if err != nil {
		return nil, err
	}
	raw, err := eng.StimulusValidate(data)
	var ee *sensorengine.Error
	if errors.As(err, &ee) {
		return nil, errf("invalid stimulus: %s", ee.Error())
	} else if err != nil {
		return nil, err
	}
	st := &stimState{def: data}
	if err := json.Unmarshal(raw, &st.rep); err != nil {
		return nil, errf("invalid stimulus: %v", err)
	}
	if st.rep.RateHz <= 0 {
		st.rep.RateHz = 50
	}
	if s.pod.sensors.stim != nil {
		s.stimStop()
	}
	s.pod.sensors.stim = st
	signals := []string{}
	for _, sg := range st.rep.Signals {
		signals = append(signals, signalName(sg.Instance, sg.Param))
	}
	return map[string]any{"program_bytes": st.rep.ProgramBytes, "signals": signals, "faults": len(st.rep.Faults),
		"duration_s": st.rep.DurationS, "repeat": st.rep.Repeat}, nil
}

// stimTarget is the running instance a stimulus signal or fault names.
func (s *Sim) stimTarget(instance, param, what string) (*sensorInst, error) {
	if instance == "" {
		var run []*sensorInst
		for _, n := range s.pod.sensors.order {
			if e := s.pod.sensors.inst[n]; e != nil && e.running {
				run = append(run, e)
			}
		}
		if len(run) != 1 {
			return nil, errf("%s: %d sensors run; name the instance (instance.%s)", what, len(run), param)
		}
		instance = run[0].name
	}
	e := s.pod.sensors.inst[instance]
	if e == nil || !e.running {
		return nil, errf("%s: no running instance %s", what, instance)
	}
	if param != "" && e.rep.param(param) == nil {
		return nil, errf("%s: %s has no such param", what, instance)
	}
	return e, nil
}

func (s *Sim) stimDrives(st *stimState, e *sensorInst) bool {
	for _, sg := range st.rep.Signals {
		if t, err := s.stimTarget(sg.Instance, sg.Param, ""); err == nil && t == e {
			return true
		}
	}
	for _, f := range st.rep.Faults {
		if t, err := s.stimTarget(f.Sensor, "", ""); err == nil && t == e {
			return true
		}
	}
	return false
}

func (s *Sim) cmdSensorStimulusStart(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	st := s.pod.sensors.stim
	if st == nil {
		return nil, fwError("no stimulus loaded: upload one with sensor_stimulus first")
	}
	for _, sg := range st.rep.Signals {
		if _, err := s.stimTarget(sg.Instance, sg.Param, "signal "+signalName(sg.Instance, sg.Param)); err != nil {
			return nil, err
		}
	}
	for i, f := range st.rep.Faults {
		e, err := s.stimTarget(f.Sensor, f.Param, fmt.Sprintf("fault %d", i))
		if err != nil {
			return nil, err
		}
		if !slices.Contains(faultKinds[f.Type], e.kind()) {
			return nil, errf("fault %d: fault %s does not apply to a %s sensor", i, f.Type, e.kind())
		}
	}
	atS, _ := r.num("at_s")
	if atS < 0 {
		return nil, fwError("at_s must be 0 or more")
	}
	now := s.now()
	s.advance(now)
	s.stimStop()
	st.running, st.applied = true, false
	st.t0 = now - int64(atS*1e9)
	s.stimApply(now)
	return map[string]any{"running": true}, nil
}

func (s *Sim) cmdSensorStimulusStop(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	s.advance(now)
	s.stimStop()
	return map[string]any{"running": false}, nil
}

// stimStop stops the stimulus and the faults it switched on; the values stay.
func (s *Sim) stimStop() {
	st := s.pod.sensors.stim
	if st == nil {
		return
	}
	st.running = false
	for _, e := range s.pod.sensors.inst {
		e.faults = slices.DeleteFunc(e.faults, func(f *sensorFault) bool { return f.stim })
	}
}

// stimTime is the stimulus time at now in seconds.
func (s *Sim) stimTime(st *stimState, now int64) float64 {
	t := float64(now-st.t0) / 1e9
	if st.rep.Repeat && st.rep.DurationS > 0 {
		return math.Mod(t, st.rep.DurationS)
	}
	return math.Min(t, st.rep.DurationS)
}

// stimApply plays the stimulus up to now, at its rate: the values and the faults it schedules.
func (s *Sim) stimApply(now int64) {
	st := s.pod.sensors.stim
	if st == nil || !st.running || st.applied && now-st.last < int64(1e9/st.rep.RateHz) {
		return
	}
	st.last, st.applied = now, true
	tS := s.stimTime(st, now)
	ended := !st.rep.Repeat && float64(now-st.t0)/1e9 >= st.rep.DurationS
	eng, err := engine()
	if err != nil {
		return
	}
	raw, err := eng.StimulusSample(st.def, tS, tS, 1)
	if err != nil {
		log.Printf("benchsim: stimulus: %v", err)
		return
	}
	var smp struct {
		Signals []struct {
			Instance string     `json:"instance"`
			Param    string     `json:"param"`
			Values   []*float64 `json:"values"`
		} `json:"signals"`
	}
	if json.Unmarshal(raw, &smp) != nil {
		return
	}
	for _, sg := range smp.Signals {
		e, err := s.stimTarget(sg.Instance, sg.Param, "")
		if err != nil || len(sg.Values) == 0 || sg.Values[0] == nil {
			continue
		}
		p := e.rep.param(sg.Param)
		v := math.Max(p.Min, math.Min(p.Max, *sg.Values[0]))
		if e.values[sg.Param] != v {
			e.values[sg.Param] = v
			s.sensorChanged(e, now)
		}
	}
	// The scheduled faults: on inside their window, off outside it.
	for i, f := range st.rep.Faults {
		e, err := s.stimTarget(f.Sensor, "", "")
		if err != nil {
			continue
		}
		on := tS >= f.AtS && (f.DurationS == nil || tS < f.AtS+*f.DurationS) && !(ended && f.DurationS != nil)
		idx := slices.IndexFunc(e.faults, func(x *sensorFault) bool { return x.stim && x.stimIdx == i })
		switch {
		case on && idx < 0:
			nf, err := newFault(e, f.Type, f.DurationS, f.Param, f.Value, f.At, f.Mask, f.RateHz)
			if err != nil {
				continue
			}
			nf.stim, nf.stimIdx = true, i
			s.addFault(e, now, nf, nil)
		case !on && idx >= 0:
			e.faults = slices.Delete(e.faults, idx, idx+1)
			s.sensorChanged(e, now)
			s.touchBase(now)
		}
	}
	if ended {
		st.running = false
	}
}
