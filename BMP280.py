import smbus


def getbarovalues(bus):
    # bmp280 address, 0x77
    # read data back from 0x88(136), 24 bytes
    b1 = bus.read_i2c_block_data(0x77, 0x88, 24)

    # convert the data
    # temp coefficents
    dig_t1 = b1[1] * 256 + b1[0]
    dig_t2 = b1[3] * 256 + b1[2]
    if dig_t2 > 32767:
        dig_t2 -= 65536
    dig_t3 = b1[5] * 256 + b1[4]
    if dig_t3 > 32767:
        dig_t3 -= 65536

    # pressure coefficents
    dig_p1 = b1[7] * 256 + b1[6]
    dig_p2 = b1[9] * 256 + b1[8]
    if dig_p2 > 32767:
        dig_p2 -= 65536
    dig_p3 = b1[11] * 256 + b1[10]
    if dig_p3 > 32767:
        dig_p3 -= 65536
    dig_p4 = b1[13] * 256 + b1[12]
    if dig_p4 > 32767:
        dig_p4 -= 65536
    dig_p5 = b1[15] * 256 + b1[14]
    if dig_p5 > 32767:
        dig_p5 -= 65536
    dig_p6 = b1[17] * 256 + b1[16]
    if dig_p6 > 32767:
        dig_p6 -= 65536
    dig_p7 = b1[19] * 256 + b1[18]
    if dig_p7 > 32767:
        dig_p7 -= 65536
    dig_p8 = b1[21] * 256 + b1[20]
    if dig_p8 > 32767:
        dig_p8 -= 65536
    dig_p9 = b1[23] * 256 + b1[22]
    if dig_p9 > 32767:
        dig_p9 -= 65536

    # bmp280 address, 0x77(118)
    # select control measurement register, 0xf4(244)
    #		0x27(39)	pressure and temperature oversampling rate = 1
    #					normal mode
    bus.write_byte_data(0x77, 0xf4, 0x27)
    # bmp280 address, 0x77(118)
    # select configuration register, 0xf5(245)
    #		0xa0(00)	stand_by time = 1000 ms
    bus.write_byte_data(0x77, 0xf5, 0xa0)

    time.sleep(0.5)

    # bmp280 address, 0x77(118)
    # read data back from 0xf7(247), 8 bytes
    # pressure msb, pressure lsb, pressure xlsb, temperature msb, temperature lsb
    # temperature xlsb, humidity msb, humidity lsb
    data = bus.read_i2c_block_data(0x77, 0xf7, 8)

    # convert pressure and temperature data to 19-bits
    adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xf0)) / 16
    adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xf0)) / 16

    # temperature offset calculations
    var1 = ((adc_t) / 16384.0 - (dig_t1) / 1024.0) * (dig_t2)
    var2 = (((adc_t) / 131072.0 - (dig_t1) / 8192.0) *
            ((adc_t) / 131072.0 - (dig_t1) / 8192.0)) * (dig_t3)
    t_fine = (var1 + var2)
    ctemp = (var1 + var2) / 5120.0

    # pressure offset calculations
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * (dig_p6) / 32768.0
    var2 = var2 + var1 * (dig_p5) * 2.0
    var2 = (var2 / 4.0) + ((dig_p4) * 65536.0)
    var1 = ((dig_p3) * var1 * var1 / 524288.0 + (dig_p2) * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * (dig_p1)
    p = 1048576.0 - adc_p
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (dig_p9) * p * p / 2147483648.0
    var2 = p * (dig_p8) / 32768.0
    pressure = (p + (var1 + var2 + (dig_p7)) / 16.0) / 100

    # units:
    #   - ctemp: celsius
    #   - pressure: hpa (hectopascal)
    return (ctemp, pressure)
