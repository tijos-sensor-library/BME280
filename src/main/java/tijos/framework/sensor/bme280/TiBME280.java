package tijos.framework.sensor.bme280;

 
import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import  tijos.util.LittleBitConverter;

/**
 * 
 * https://os.mbed.com/users/MACRUM/code/BME280/docs/tip/
 * https://os.mbed.com/users/loopsva/code/bme280/file/96075bee19f0/bme280.cpp/
 * @author TiJOS
 *
 */



class TiBME280Register{
	
	public static final int BME280_CHIP_ID_REG    			=0xD0;  //Chip ID Register 
	public static final int  BME280_CAL_DATA_START_1        =     0x88;  //Calibration data parameters
	public static final int  BME280_CAL_DATA_START_2        =     0xE1;  //More calibration data parameters
	public static final int  BME280_RST_REG                 =     0xE0;  //Softreset Register 
	public static final int  BME280_STAT_REG                =     0xF3;  //Status Register 
	public static final int  BME280_CTRL_MEAS_REG           =     0xF4;  //Ctrl Measure Register 
	public static final int  BME280_CTRL_HUMIDITY_REG       =     0xF2;  //Ctrl Humidity Register
	public static final int  BME280_CONFIG_REG              =     0xF5;  //Configuration Register 
	public static final int  BME280_PRESSURE_MSB_REG        =     0xF7;  //Pressure MSB Register 
	public static final int  BME280_PRESSURE_LSB_REG        =     0xF8;  //Pressure LSB Register 
	public static final int  BME280_PRESSURE_XLSB_REG       =     0xF9;  //Pressure XLSB Register 
	public static final int  BME280_TEMPERATURE_MSB_REG     =     0xFA;  //Temperature MSB Reg 
	public static final int  BME280_TEMPERATURE_LSB_REG     =     0xFB;  //Temperature LSB Reg 
	public static final int  BME280_TEMPERATURE_XLSB_REG    =     0xFC;  //Temperature XLSB Reg 
	public static final int  BME280_HUMIDITY_MSB_REG        =     0xFD;  //Humidity MSB Reg 
	public static final int  BME280_HUMIDITY_LSB_REG        =     0xFE;  //Humidity LSB Reg 
}


public class TiBME280 {
	/**
	 * TiI2CMaster object
	 */
	private TiI2CMaster i2cmObj;

	private int i2cSlaveAddress = 0x76;
	
	byte[] data = new byte[4];

	int dig_T1;
	int dig_T2, dig_T3;
	int dig_P1;
	int dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	int dig_H1, dig_H3;
	int dig_H2, dig_H4, dig_H5, dig_H6;
	long t_fine;

	public TiBME280(TiI2CMaster i2c) {
		this(i2c, 0x76);
	}

	public TiBME280(TiI2CMaster i2c, int address) {
		this.i2cmObj = i2c;
		this.i2cSlaveAddress = address;

	}

	/**
	 * Initialize BME280 sensor
	 *
	 * Configure sensor setting and read parameters for calibration
	 * @throws IOException 
	 *
	 */
	public void initialize() throws IOException {
		byte[] cmd = new byte[18];

		cmd[0] = (byte) 0xf2; // ctrl_hum
		cmd[1] = 0x01; // Humidity oversampling x1
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 2);

		cmd[0] = (byte) 0xf4; // ctrl_meas
		cmd[1] = 0x27; // Temperature oversampling x1, Pressure oversampling x1,
						// Normal mode
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 2);

		cmd[0] = (byte) 0xf5; // Configure
		cmd[1] = (byte) 0xa0; // Standby 1000ms, Filter off
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 2);

		cmd[0] = (byte) 0x88; // read dig_T regs
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, cmd, 0, 6);

		dig_T1 = LittleBitConverter.ToUInt16(cmd, 0);
		dig_T2 = LittleBitConverter.ToInt16(cmd, 2);
		dig_T3 = LittleBitConverter.ToInt16(cmd, 4);

		cmd[0] = (byte) 0x8E; // read dig_P regs
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, cmd, 0, 18);

		dig_P1 = LittleBitConverter.ToUInt16(cmd, 0); 
		dig_P2 = LittleBitConverter.ToInt16(cmd, 2); 
		dig_P3 = LittleBitConverter.ToInt16(cmd, 4); 
		dig_P4 = LittleBitConverter.ToInt16(cmd, 6); 
		dig_P5 = LittleBitConverter.ToInt16(cmd, 8); 
		dig_P6 = LittleBitConverter.ToInt16(cmd, 10);
		dig_P7 = LittleBitConverter.ToInt16(cmd, 12);
		dig_P8 = LittleBitConverter.ToInt16(cmd, 14);
		dig_P9 = LittleBitConverter.ToInt16(cmd, 16);

		cmd[0] = (byte) 0xA1; // read dig_H regs
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, cmd, 0, 1);

		cmd[1] = (byte) 0xE1; // read dig_H regs
		this.i2cmObj.write(i2cSlaveAddress, cmd, 1, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, cmd, 1, 7);

		dig_H1 = cmd[0];
		dig_H2 = LittleBitConverter.ToInt16(cmd, 1);
		dig_H3 = (byte)(cmd[3] & 0xFF);
		dig_H4 = ((cmd[4]&0xFF) << 4) | (cmd[5] & 0x0f);
		dig_H5 = ((cmd[6]&0xFF) << 4) | (((cmd[5]&0xFF) >>> 4) & 0x0f);
		
		dig_H6 = cmd[7];

	}
	
	
	public int getDevID() throws IOException 
	{
		data[0] = (byte)TiBME280Register.BME280_CHIP_ID_REG;
		this.i2cmObj.write(i2cSlaveAddress, data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, data, 0, 1);
		
		return data[0]&0xFF;
	}

	/**
	 * Read the current temperature value (degree Celsius) from BME280 sensor
	 * @throws IOException 
	 *
	 */
	public double getTemperature() throws IOException {
		long temp_raw;
		

		data[0] = (byte) 0xfa; // temp_msb

		this.i2cmObj.write(i2cSlaveAddress, data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, data, 1, 3);

		temp_raw = ((data[1] & 0xFF) << 12) 
				| ((data[2] & 0xFF) << 4) 
				| ((data[3] & 0xFF) >> 4);

		long temp = (((((temp_raw >> 3) - (dig_T1 << 1))) * dig_T2) >> 11)
				+ ((((((temp_raw >> 4) - dig_T1) * ((temp_raw >> 4) - dig_T1)) >> 12) * dig_T3) >> 14);

		t_fine = temp;
		temp = (temp * 5 + 128) >> 8;

		return (temp / 100.0);
	}

	/**
	 * Read the current pressure value (hectopascal)from BME280 sensor
	 * @throws IOException 
	 *
	 */
	public double getPressure() throws IOException {
		long press_raw;		

		data[0] = (byte) 0xf7; // press_msb
		this.i2cmObj.write(i2cSlaveAddress, data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, data, 1, 3);

		press_raw = ((data[1] & 0xFF) << 12)
				| ((data[2] & 0xFF) << 4) 
				| ((data[3] & 0xFF) >> 4);

		long var1, var2;
		long press;

		var1 = (t_fine >> 1) - 64000;
		var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * dig_P6;
		var2 = var2 + ((var1 * dig_P5) << 1);
		var2 = (var2 >> 2) + (dig_P4 << 16);
		var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((dig_P2 * var1) >> 1)) >> 18;
		var1 = ((32768 + var1) * dig_P1) >> 15;
		if (var1 == 0) {
			return 0;
		}

		press = (((1048576 - press_raw) - (var2 >> 12))) * 3125;
		if (press < 0x80000000) {
			press = (press << 1) / var1;
		} else {
			press = (press / var1) * 2;
		}
		var1 = ((int) dig_P9 * ((int) (((press >> 3) * (press >> 3)) >> 13))) >> 12;
		var2 = (((int) (press >> 2)) * (int) dig_P8) >> 13;
		press = (press + ((var1 + var2 + dig_P7) >> 4));

		return (press / 100.0f);
	}

	/**
	 * Read the current humidity value (humidity %) from BME280 sensor
	 * @throws IOException 
	 *
	 */
	public double getHumidity() throws IOException {
		long hum_raw;
		double humf;	

		data[0] = (byte) 0xfd; // hum_msb
		this.i2cmObj.write(i2cSlaveAddress, data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, data, 1, 2);

		hum_raw = ((data[1] & 0xFF) << 8) 
				| (data[2] & 0xFF);

		long v_x1;

		v_x1 = t_fine - 76800;
		v_x1 = (((((hum_raw << 14) - (((int) dig_H4) << 20) - (((int) dig_H5) * v_x1)) + ((int) 16384)) >> 15)
				* (((((((v_x1 * (int) dig_H6) >> 10) * (((v_x1 * ((int) dig_H3)) >> 11) + 32768)) >> 10) + 2097152)
						* (int) dig_H2 + 8192) >> 14));
		v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * (int) dig_H1) >> 4));
		v_x1 = (v_x1 < 0 ? 0 : v_x1);
		v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);

		humf = (double) (v_x1 >> 12);

		return (humf / 1024.0f);
	}

}
