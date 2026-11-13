// Package sensorengine runs the BenchPod sensor definition engine, compiled to WebAssembly
// (benchpod-firmware stm32h563/wasm, ABI in its README.md), with wazero, a pure-Go runtime. It is
// the C the pod compiles and plays definitions with, so the simulated pod answers the DUT with
// the register images and UART messages a real pod would send.
//
// sensor_def.wasm is a vendored build; scripts/update-sensor-engine.sh refreshes it and records
// its sha256 in sensor_def.wasm.sha256 (the test checks the two match). The EmbeddedCI server
// vendors the same file.
package sensorengine

import (
	"context"
	"crypto/sha256"
	_ "embed"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"strings"
	"sync"
	"time"

	"github.com/tetratelabs/wazero"
	"github.com/tetratelabs/wazero/api"
)

//go:embed sensor_def.wasm
var moduleBytes []byte

//go:embed sensor_def.wasm.sha256
var moduleRecord string

// ABIVersion is the engine ABI this package speaks.
const ABIVersion = 1

// callTimeout bounds one engine call. The engine has its own step limits; this is a backstop.
const callTimeout = 5 * time.Second

// maxIdle is how many instances stay ready between calls (each holds about 7 MB of memory).
const maxIdle = 4

// Engine is the compiled module plus a pool of instances. An instance runs one call at a time;
// concurrent calls take separate instances, so an Engine is safe for concurrent use.
type Engine struct {
	rt       wazero.Runtime
	compiled wazero.CompiledModule
	idle     chan api.Module
}

// New compiles the embedded module.
func New(ctx context.Context) (*Engine, error) {
	rt := wazero.NewRuntimeWithConfig(ctx, wazero.NewRuntimeConfig().WithCloseOnContextDone(true))
	compiled, err := rt.CompileModule(ctx, moduleBytes)
	if err != nil {
		_ = rt.Close(ctx)
		return nil, fmt.Errorf("sensor_def.wasm: %w", err)
	}
	e := &Engine{rt: rt, compiled: compiled, idle: make(chan api.Module, maxIdle)}
	m, err := e.get(ctx)
	if err != nil {
		_ = rt.Close(ctx)
		return nil, err
	}
	res, err := m.ExportedFunction("abi_version").Call(ctx)
	if err != nil || len(res) != 1 || res[0] != ABIVersion {
		_ = rt.Close(ctx)
		return nil, fmt.Errorf("sensor_def.wasm: ABI %v (%v), this package speaks %d", res, err, ABIVersion)
	}
	e.put(m)
	return e, nil
}

var (
	sharedOnce sync.Once
	shared     *Engine
	sharedErr  error
)

// Shared is one Engine for the whole process, compiled on first use (compiling takes a moment,
// and every simulated pod can share it).
func Shared() (*Engine, error) {
	sharedOnce.Do(func() { shared, sharedErr = New(context.Background()) })
	return shared, sharedErr
}

// Close frees the runtime.
func (e *Engine) Close(ctx context.Context) error { return e.rt.Close(ctx) }

// ModuleSHA256 is the sha256 of the embedded module (hex).
func ModuleSHA256() string {
	sum := sha256.Sum256(moduleBytes)
	return hex.EncodeToString(sum[:])
}

// ModuleRecord is the recorded sha256 line: "<sha256>  sensor_def.wasm  benchpod-firmware <commit>".
func ModuleRecord() string { return strings.TrimSpace(moduleRecord) }

func (e *Engine) get(ctx context.Context) (api.Module, error) {
	select {
	case m := <-e.idle:
		return m, nil
	default:
	}
	// An empty name lets the runtime hold any number of instances of the module.
	m, err := e.rt.InstantiateModule(ctx, e.compiled, wazero.NewModuleConfig().WithName(""))
	if err != nil {
		return nil, fmt.Errorf("sensor_def.wasm: instantiate: %w", err)
	}
	return m, nil
}

func (e *Engine) put(m api.Module) {
	select {
	case e.idle <- m:
	default:
		_ = m.Close(context.Background())
	}
}

// arg is one call argument: bytes copied into the module (pointer and length), or a float.
type arg struct {
	bytes []byte
	isBuf bool
	f     float64
}

func buf(b []byte) arg  { return arg{bytes: b, isBuf: true} }
func num(f float64) arg { return arg{f: f} }

// call runs an export and returns the JSON it wrote.
func (e *Engine) call(fn string, args ...arg) ([]byte, error) {
	ctx, cancel := context.WithTimeout(context.Background(), callTimeout)
	defer cancel()
	m, err := e.get(ctx)
	if err != nil {
		return nil, err
	}
	ok := false
	defer func() {
		if ok {
			e.put(m)
		} else {
			_ = m.Close(context.Background()) // a trap or a timeout: do not reuse it
		}
	}()
	alloc, free := m.ExportedFunction("sd_alloc"), m.ExportedFunction("sd_free")
	var params, ptrs []uint64
	defer func() {
		for _, p := range ptrs {
			_, _ = free.Call(context.Background(), p)
		}
	}()
	for _, a := range args {
		if !a.isBuf {
			params = append(params, api.EncodeF64(a.f))
			continue
		}
		if a.bytes == nil {
			params = append(params, 0, 0)
			continue
		}
		res, err := alloc.Call(ctx, uint64(len(a.bytes)))
		if err != nil {
			return nil, fmt.Errorf("sensor_def.wasm: sd_alloc: %w", err)
		}
		p := res[0]
		if p == 0 {
			return nil, errors.New("sensor_def.wasm: out of memory")
		}
		ptrs = append(ptrs, p)
		if !m.Memory().Write(uint32(p), a.bytes) {
			return nil, errors.New("sensor_def.wasm: write outside memory")
		}
		params = append(params, p, uint64(len(a.bytes)))
	}
	res, err := m.ExportedFunction(fn).Call(ctx, params...)
	if err != nil {
		return nil, fmt.Errorf("sensor_def.wasm %s: %w", fn, err)
	}
	outp, err := m.ExportedFunction("out_ptr").Call(ctx)
	if err != nil {
		return nil, fmt.Errorf("sensor_def.wasm out_ptr: %w", err)
	}
	out, okRead := m.Memory().Read(uint32(outp[0]), uint32(res[0]))
	if !okRead {
		return nil, errors.New("sensor_def.wasm: result outside memory")
	}
	ok = true
	return append([]byte(nil), out...), nil
}

// Error is the engine refusing a definition, a stimulus or an argument. Error() is the engine's
// own text, the one the pod replies with: "registers[4].s16be: unknown name 'ax' at 17".
type Error struct {
	Text    string `json:"error"`
	Path    string `json:"path"`
	Message string `json:"message"`
}

func (e *Error) Error() string { return e.Text }

// check returns out, or the engine's error when it answered {"ok": false}.
func check(out []byte) (json.RawMessage, error) {
	var r struct {
		OK bool `json:"ok"`
		Error
	}
	if err := json.Unmarshal(out, &r); err != nil {
		return nil, fmt.Errorf("sensor_def.wasm: bad answer: %w", err)
	}
	if !r.OK {
		if r.Text == "" {
			r.Text = r.Message
		}
		return nil, &r.Error
	}
	return out, nil
}

// Validate compiles a definition and returns the engine's report (kind, name, params, bus...).
func (e *Engine) Validate(def []byte) (json.RawMessage, error) {
	out, err := e.call("validate", buf(def))
	if err != nil {
		return nil, err
	}
	return check(out)
}

// Preview is what the sensor puts out at tMs for values (a JSON object, nil for the defaults):
// an I2C/SPI image (from live, the image the DUT wrote, or from power on when live is nil), a
// UART message or an analog value (input is the measured voltage for excitation sensors).
func (e *Engine) Preview(def, values, live []byte, tMs, input float64) (json.RawMessage, error) {
	out, err := e.call("preview", buf(def), buf(values), buf(live), num(tMs), num(input))
	if err != nil {
		return nil, err
	}
	return check(out)
}

// UARTAnswer feeds request bytes to a UART sensor at tMs and lists its answers.
func (e *Engine) UARTAnswer(def, values, request []byte, tMs float64) (json.RawMessage, error) {
	out, err := e.call("uart_answer", buf(def), buf(values), buf(request), num(tMs))
	if err != nil {
		return nil, err
	}
	return check(out)
}

// StimulusValidate compiles a stimulus and returns its report (signals, faults, duration...).
func (e *Engine) StimulusValidate(stim []byte) (json.RawMessage, error) {
	out, err := e.call("stimulus_validate", buf(stim))
	if err != nil {
		return nil, err
	}
	return check(out)
}

// StimulusSample returns the stimulus values at fromS + k*stepS up to toS.
func (e *Engine) StimulusSample(stim []byte, fromS, toS, stepS float64) (json.RawMessage, error) {
	out, err := e.call("stimulus_sample", buf(stim), num(fromS), num(toS), num(stepS))
	if err != nil {
		return nil, err
	}
	return check(out)
}
