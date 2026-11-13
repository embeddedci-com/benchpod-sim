package sensorengine

import (
	"encoding/json"
	"errors"
	"os"
	"strings"
	"testing"
)

func TestModuleMatchesRecord(t *testing.T) {
	if f := strings.Fields(ModuleRecord()); len(f) < 2 || f[0] != ModuleSHA256() || f[1] != "sensor_def.wasm" {
		t.Fatalf("sensor_def.wasm sha256 %s does not match the record %q (refresh with scripts/update-sensor-engine.sh)", ModuleSHA256(), ModuleRecord())
	}
}

func TestValidateAndPreview(t *testing.T) {
	e, err := Shared()
	if err != nil {
		t.Fatal(err)
	}
	def, err := os.ReadFile("../benchsim/testdata/sensors/bmp280.json")
	if err != nil {
		t.Fatal(err)
	}
	rep, err := e.Validate(def)
	if err != nil {
		t.Fatal(err)
	}
	var v struct {
		Kind, Name string
		Bus        struct{ Address int }
	}
	_ = json.Unmarshal(rep, &v)
	if v.Kind != "i2c" || v.Name != "bmp280" || v.Bus.Address != 0x76 {
		t.Fatalf("validate: %s", rep)
	}
	pv, err := e.Preview(def, []byte(`{"temperature_c":30}`), nil, 0, 0)
	if err != nil {
		t.Fatal(err)
	}
	if !strings.Contains(string(pv), `"image_b64"`) {
		t.Fatalf("preview: %s", pv)
	}
	_, err = e.Validate([]byte(`{"format":1,"kind":"i2c","name":"x","params":[],"i2c":{"address":118},"registers":[{"at":0,"u8":"nope"}]}`))
	var ee *Error
	if !errors.As(err, &ee) || ee.Error() == "" {
		t.Fatalf("broken definition: %v", err)
	}
}
