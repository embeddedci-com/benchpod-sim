package benchsim

// The node's side of the BMP280: it reads the image back with the Bosch datasheet integer code (a
// port of the node's bmp280_comp.h). The pod's side, the register image, comes from the sensor
// definition engine (sensors.go), so the readings are what the real pair produces, rounding
// included.

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
