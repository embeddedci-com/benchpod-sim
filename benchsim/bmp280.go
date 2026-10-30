package benchsim

import "math"

// The BMP280/BME280 compensation math, both directions: the pod's emulated part inverts it to
// build its register image (a port of benchpod-firmware sensor_bmp280.c), and the node reads the
// image back with the Bosch datasheet integer code (a port of the node's bmp280_comp.h). Running
// both keeps the readings exactly what the real pair produces, rounding included.

// The emulated part's fixed trim (sensor_bmp280.c).
var bmpTrim = struct {
	t1         uint16
	t2, t3     int16
	p1         uint16
	p2, p3, p4 int16
	p5, p6, p7 int16
	p8, p9     int16
	h1, h3     uint8
	h2, h4, h5 int16
	h6         int8
}{
	t1: 27504, t2: 26435, t3: -1000,
	p1: 36477, p2: -10685, p3: 3024, p4: 2855, p5: 140, p6: -7, p7: 15500, p8: -14600, p9: 6000,
	h1: 75, h2: 362, h3: 0, h4: 322, h5: 50, h6: 30,
}

// bmpCalib is the 24 trim bytes from 0x88 as the node parses them.
type bmpCalib struct {
	t1                             uint16
	t2, t3                         int16
	p1                             uint16
	p2, p3, p4, p5, p6, p7, p8, p9 int16
}

func parseBmpCalib(b []byte) bmpCalib {
	u16 := func(i int) uint16 { return uint16(b[i]) | uint16(b[i+1])<<8 }
	return bmpCalib{
		t1: u16(0), t2: int16(u16(2)), t3: int16(u16(4)),
		p1: u16(6), p2: int16(u16(8)), p3: int16(u16(10)), p4: int16(u16(12)), p5: int16(u16(14)),
		p6: int16(u16(16)), p7: int16(u16(18)), p8: int16(u16(20)), p9: int16(u16(22)),
	}
}

// bmpTempCdc is bmp280_temp_cdc: temperature in 0.01 degC, and t_fine.
func bmpTempCdc(c bmpCalib, adcT int32) (int32, int32) {
	var1 := (((adcT >> 3) - (int32(c.t1) << 1)) * int32(c.t2)) >> 11
	d := (adcT >> 4) - int32(c.t1)
	var2 := (((d * d) >> 12) * int32(c.t3)) >> 14
	tFine := var1 + var2
	return (tFine*5 + 128) >> 8, tFine
}

// bmpPressPa is bmp280_press_pa: pressure in Pa, 0 when the trim is unusable.
func bmpPressPa(c bmpCalib, adcP, tFine int32) uint32 {
	var1 := int64(tFine) - 128000
	var2 := var1 * var1 * int64(c.p6)
	var2 += (var1 * int64(c.p5)) * 131072
	var2 += int64(c.p4) * 34359738368
	var1 = (var1*var1*int64(c.p3))/256 + (var1*int64(c.p2))*4096
	var1 = (140737488355328 + var1) * int64(c.p1) / 8589934592
	if var1 == 0 {
		return 0
	}
	p := int64(1048576 - adcP)
	p = ((p*2147483648 - var2) * 3125) / var1
	var1 = (int64(c.p9) * (p / 8192) * (p / 8192)) / 33554432
	var2 = (int64(c.p8) * p) / 524288
	p = (p+var1+var2)/256 + int64(c.p7)*16
	return uint32(p / 256)
}

// The emulated part's own (float) compensation, used for the inversion (sensor_bmp280.c).
func simTempCenti(adcT int32) (int32, int32) {
	t := bmpTrim
	var1 := (((adcT >> 3) - (int32(t.t1) << 1)) * int32(t.t2)) >> 11
	var2 := ((((adcT >> 4) - int32(t.t1)) * ((adcT >> 4) - int32(t.t1)) >> 12) * int32(t.t3)) >> 14
	tFine := var1 + var2
	return (tFine*5 + 128) >> 8, tFine
}

func simPressPa(adcP, tFine int32) float64 {
	t := bmpTrim
	tf := float64(tFine)
	var1 := tf - 128000.0
	var2 := var1 * var1 * float64(t.p6)
	var2 += var1 * float64(t.p5) * 131072.0
	var2 += float64(t.p4) * 34359738368.0
	var1 = (var1*var1*float64(t.p3))/256.0 + var1*float64(t.p2)*4096.0
	var1 = ((140737488355328.0 + var1) * float64(t.p1)) / 8589934592.0
	if var1 == 0 {
		return 0
	}
	p := float64(1048576 - adcP)
	p = ((p*2147483648.0 - var2) * 3125.0) / var1
	pr := math.Trunc(p / 8192.0)
	var1 = math.Trunc(float64(t.p9) * pr * pr / 33554432.0)
	var2 = math.Trunc(float64(t.p8) * p / 524288.0)
	p = math.Trunc((p+var1+var2)/256.0) + float64(t.p7)*16.0
	return float64(float32(p / 256.0))
}

func findAdcT(targetC float64) int32 {
	target := int32(math.Round(targetC * 100))
	lo, hi, best, bestErr := int32(0), int32(0xFFFFF), int32(0), int32(math.MaxInt32)
	for lo <= hi {
		mid := lo + (hi-lo)/2
		tc, _ := simTempCenti(mid)
		e := tc - target
		if e < 0 {
			e = -e
		}
		if e < bestErr {
			bestErr, best = e, mid
		}
		switch {
		case tc < target:
			lo = mid + 1
		case tc > target:
			hi = mid - 1
		default:
			return mid
		}
	}
	return best
}

func findAdcP(targetPa float64, adcT int32) int32 {
	_, tFine := simTempCenti(adcT)
	lo, hi, best, bestErr := int32(0), int32(0xFFFFF), int32(0), math.Inf(1)
	for lo <= hi {
		mid := lo + (hi-lo)/2
		pa := simPressPa(mid, tFine)
		if math.IsNaN(pa) || math.IsInf(pa, 0) {
			hi = mid - 1
			continue
		}
		e := math.Abs(pa - targetPa)
		if e < bestErr {
			bestErr, best = e, mid
		}
		switch {
		case pa > targetPa:
			lo = mid + 1
		case pa < targetPa:
			hi = mid - 1
		default:
			return mid
		}
	}
	return best
}

// bmeHumQ10 is the BME280 integer humidity compensation (%RH x 1024).
func bmeHumQ10(adcH, tFine int32) uint32 {
	t := bmpTrim
	v := tFine - 76800
	v = ((((adcH << 14) - (int32(t.h4) << 20) - (int32(t.h5) * v)) + 16384) >> 15) *
		(((((((v*int32(t.h6))>>10)*(((v*int32(t.h3))>>11)+32768))>>10)+2097152)*int32(t.h2) + 8192) >> 14)
	v = v - (((((v >> 15) * (v >> 15)) >> 7) * int32(t.h1)) >> 4)
	if v < 0 {
		v = 0
	}
	if v > 419430400 {
		v = 419430400
	}
	return uint32(v >> 12)
}

func findAdcH(targetPct float64, adcT int32) int32 {
	_, tFine := simTempCenti(adcT)
	target := int32(math.Round(targetPct * 1024))
	lo, hi, best, bestErr := int32(0), int32(0xFFFF), int32(0), int32(math.MaxInt32)
	for lo <= hi {
		mid := lo + (hi-lo)/2
		h := int32(bmeHumQ10(mid, tFine))
		e := h - target
		if e < 0 {
			e = -e
		}
		if e < bestErr {
			bestErr, best = e, mid
		}
		switch {
		case h < target:
			lo = mid + 1
		case h > target:
			hi = mid - 1
		default:
			return mid
		}
	}
	return best
}

// buildBmpImage fills the register image of a BMP280 (chip id 0x58) or BME280 (0x60) reading
// temperature/pressure (and humidity on the BME280).
func buildBmpImage(img *[256]byte, bme bool, tempC, pressPa, humPct float64) {
	*img = [256]byte{}
	if bme {
		img[0xD0] = 0x60
	} else {
		img[0xD0] = 0x58
	}
	t := bmpTrim
	put := func(a int, v uint16) { img[a], img[a+1] = byte(v), byte(v>>8) }
	put(0x88, t.t1)
	put(0x8A, uint16(t.t2))
	put(0x8C, uint16(t.t3))
	put(0x8E, t.p1)
	put(0x90, uint16(t.p2))
	put(0x92, uint16(t.p3))
	put(0x94, uint16(t.p4))
	put(0x96, uint16(t.p5))
	put(0x98, uint16(t.p6))
	put(0x9A, uint16(t.p7))
	put(0x9C, uint16(t.p8))
	put(0x9E, uint16(t.p9))
	adcT := findAdcT(tempC) & 0xFFFFF
	adcP := findAdcP(pressPa, adcT) & 0xFFFFF
	img[0xF7] = byte(adcP >> 12)
	img[0xF8] = byte(adcP >> 4)
	img[0xF9] = byte((adcP & 0x0F) << 4)
	img[0xFA] = byte(adcT >> 12)
	img[0xFB] = byte(adcT >> 4)
	img[0xFC] = byte((adcT & 0x0F) << 4)
	if bme {
		img[0xA1] = t.h1
		put(0xE1, uint16(t.h2))
		img[0xE3] = t.h3
		img[0xE4] = byte(t.h4 >> 4)
		img[0xE5] = byte((t.h5&0x0F)<<4 | t.h4&0x0F)
		img[0xE6] = byte(t.h5 >> 4)
		img[0xE7] = byte(t.h6)
		adcH := findAdcH(humPct, adcT)
		img[0xFD] = byte(adcH >> 8)
		img[0xFE] = byte(adcH)
	}
}
