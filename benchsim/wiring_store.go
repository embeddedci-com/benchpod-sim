package benchsim

import (
	"encoding/base64"
	"encoding/json"
	"regexp"
	"sort"
)

// The pod's wiring profiles (firmware wiring_store.c; API.md "Wiring profiles"): named JSON
// objects the pod keeps in flash, so they survive a reboot. Firmware 3.10.8 and later stores them,
// and firmware 4.0's features_rev tells the server so: the server then saves a device's wiring on
// the pod with wiring_put instead of in its own database.

const (
	wiringMaxProfiles = 16
	wiringMaxBytes    = 4096      // WIRING_PROFILE_MAX
	wiringTotalBytes  = 16 * 1024 // the store's space for every profile together
	wiringPieceMax    = 600       // WIRING_PIECE_MAX, raw bytes per piece
)

// The wiring store's error codes (errors/catalog.toml).
const (
	codeMissingArgument     = 6
	codeWiringNotFound      = 7
	codeWiringBadName       = 8
	codeWiringRevConflict   = 9
	codeWiringStoreFull     = 10
	codeWiringTooLarge      = 11
	codeWiringNotJSONObject = 12
	codeWiringDeleteDefault = 13
	codeWiringOutOfOrder    = 14
)

var wiringNameRe = regexp.MustCompile(`^[a-z0-9][a-z0-9-]{0,39}$`)

type wiringProfile struct {
	rev  int
	data []byte
}

// wiringStore lives in podFlash: a reboot keeps it.
type wiringStore struct {
	profiles map[string]*wiringProfile
	def      string
	// the one upload in progress for the whole pod
	upName  string
	upTotal int
	upBuf   []byte
}

func (w *wiringStore) used(except string) int {
	n := 0
	for name, p := range w.profiles {
		if name != except {
			n += len(p.data)
		}
	}
	return n
}

func wiringName(r request) (string, error) {
	if !r.has("name") {
		return "", coded(codeMissingArgument, "arg", "name")
	}
	name := r.str("name")
	if !wiringNameRe.MatchString(name) {
		return "", coded(codeWiringBadName, "name", name)
	}
	return name, nil
}

func (s *Sim) cmdWiringList(request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	w := &s.flash.wiring
	names := make([]string, 0, len(w.profiles))
	for name := range w.profiles {
		names = append(names, name)
	}
	sort.Strings(names)
	list := make([]map[string]any, 0, len(names))
	for _, name := range names {
		p := w.profiles[name]
		list = append(list, map[string]any{"name": name, "rev": p.rev, "bytes": len(p.data)})
	}
	var def any
	if w.def != "" {
		def = w.def
	}
	return map[string]any{"default": def, "profiles": list, "max_profiles": wiringMaxProfiles,
		"max_bytes": wiringMaxBytes, "free_bytes": wiringTotalBytes - w.used("")}, nil
}

func (s *Sim) cmdWiringGet(r request) (any, error) {
	name, err := wiringName(r)
	if err != nil {
		return nil, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	p, ok := s.flash.wiring.profiles[name]
	if !ok {
		return nil, coded(codeWiringNotFound, "name", name)
	}
	off := min(max(r.int("offset", 0), 0), len(p.data))
	end := min(off+wiringPieceMax, len(p.data))
	return map[string]any{"name": name, "rev": p.rev, "total": len(p.data), "offset": off,
		"data_b64": base64.RawURLEncoding.EncodeToString(p.data[off:end])}, nil
}

func (s *Sim) cmdWiringPut(r request) (any, error) {
	name, err := wiringName(r)
	if err != nil {
		return nil, err
	}
	for _, k := range []string{"offset", "total", "data_b64"} {
		if !r.has(k) {
			return nil, coded(codeMissingArgument, "arg", k)
		}
	}
	off, total := r.int("offset", 0), r.int("total", 0)
	piece, err := base64.RawURLEncoding.DecodeString(r.str("data_b64"))
	if err != nil {
		return nil, coded(codeWiringNotJSONObject)
	}
	if total > wiringMaxBytes {
		return nil, coded(codeWiringTooLarge, "bytes", total, "max", wiringMaxBytes)
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	w := &s.flash.wiring
	if w.profiles == nil {
		w.profiles = map[string]*wiringProfile{}
	}
	if off == 0 {
		cur, exists := w.profiles[name]
		if r.has("if_revision") {
			want := r.int("if_revision", 0)
			have := 0
			if exists {
				have = cur.rev
			}
			if want != have {
				return nil, coded(codeWiringRevConflict, "name", name, "rev", have)
			}
		}
		if !exists && len(w.profiles) >= wiringMaxProfiles {
			return nil, coded(codeWiringStoreFull, "max", wiringMaxProfiles)
		}
		w.upName, w.upTotal, w.upBuf = name, total, nil
	} else if w.upName != name || w.upTotal != total || off != len(w.upBuf) {
		return nil, coded(codeWiringOutOfOrder, "expected", len(w.upBuf))
	}
	w.upBuf = append(w.upBuf, piece...)
	if len(w.upBuf) > total {
		w.upName, w.upBuf = "", nil
		return nil, coded(codeWiringOutOfOrder, "expected", total)
	}
	if len(w.upBuf) < total {
		return map[string]any{"name": name, "received": len(w.upBuf), "total": total}, nil
	}
	data := w.upBuf
	w.upName, w.upBuf = "", nil
	var obj map[string]any
	if json.Unmarshal(data, &obj) != nil || obj == nil {
		return nil, coded(codeWiringNotJSONObject)
	}
	if w.used(name)+len(data) > wiringTotalBytes {
		return nil, coded(codeWiringStoreFull, "max", wiringMaxProfiles)
	}
	p, ok := w.profiles[name]
	if !ok {
		p = &wiringProfile{}
		w.profiles[name] = p
	}
	p.rev++
	p.data = data
	if w.def == "" {
		w.def = name
	}
	return map[string]any{"name": name, "rev": p.rev, "default": w.def == name}, nil
}

func (s *Sim) cmdWiringDelete(r request) (any, error) {
	name, err := wiringName(r)
	if err != nil {
		return nil, err
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	w := &s.flash.wiring
	if _, ok := w.profiles[name]; !ok {
		return nil, coded(codeWiringNotFound, "name", name)
	}
	if w.def == name {
		return nil, coded(codeWiringDeleteDefault, "name", name)
	}
	delete(w.profiles, name)
	return map[string]any{"name": name, "deleted": true}, nil
}

func (s *Sim) cmdWiringDefault(r request) (any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	w := &s.flash.wiring
	if r.has("name") {
		name := r.str("name")
		if !wiringNameRe.MatchString(name) {
			return nil, coded(codeWiringBadName, "name", name)
		}
		if _, ok := w.profiles[name]; !ok {
			return nil, coded(codeWiringNotFound, "name", name)
		}
		w.def = name
	}
	var def any
	if w.def != "" {
		def = w.def
	}
	return map[string]any{"default": def}, nil
}
