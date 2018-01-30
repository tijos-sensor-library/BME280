package tijos.framework.sensor.bme280;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.util.Delay;
import tijos.util.LittleBitConverter;

/**
 * BOSCH BME280 Combined humidity and pressure sensor library for TiJOS
 * 
 * @author TiJOS
 *
 */

class TiBME280Register {

	// Chip ID Register
	public static final int BME280_CHIP_ID_REG = 0xD0;

	// Calibration data parameters
	public static final int BME280_CAL_DATA_START_1 = 0x88;

	// More calibration data parameters
	public static final int BME280_CAL_DATA_START_2 = 0xE1;

	// Softreset Register
	public static final int BME280_RST_REG = 0xE0;

	// Status Register
	public static final int BME280_STAT_REG = 0xF3;

	// Ctrl Measure Register
	public static final int BME280_CTRL_MEAS_REG = 0xF4;

	// Ctrl Humidity Register
	public static final int BME280_CTRL_HUMIDITY_REG = 0xF2;

	// Configuration Register
	public static final int BME280_CONFIG_REG = 0xF5;

	// Pressure MSB Register
	public static final int BME280_PRESSURE_MSB_REG = 0xF7;

	// Pressure LSB Register
	public static final int BME280_PRESSURE_LSB_REG = 0xF8;

	// Pressure XLSB Register
	public static final int BME280_PRESSURE_XLSB_REG = 0xF9;

	// Temperature MSB Reg
	public static final int BME280_TEMPERATURE_MSB_REG = 0xFA;

	// Temperature LSB Reg
	public static final int BME280_TEMPERATURE_LSB_REG = 0xFB;

	// Temperature XLSB Reg
	public static final int BME280_TEMPERATURE_XLSB_REG = 0xFC;

	// Humidity MSB Reg
	public static final int BME280_HUMIDITY_MSB_REG = 0xFD;

	// Humidity LSB Reg
	public static final int BME280_HUMIDITY_LSB_REG = 0xFE;
}

public class TiBME280 {

	private static final int BME280_I2C_ADDRESS = 0X77;

	private static final int BME280_CHIP_ID = 0x60;

	public enum Mode {
		Sleep, Forced, Forced2, Normal;
	}

	public enum OverSampling {
		SAMPLING_NO, SAMPLING_1X, SAMPLING_2X, SAMPLING_4X, SAMPLING_8X, SAMPLING_16X;
	}

	public enum StandByTime {
		TIME_1_MS, TIME_62_5_MS, TIME_125_MS, TIME_250_MS, TIME_500_MS, TIME_1000_MS, TIME_10_MS, TIME_20_MS;
	}

	public enum FilterCoeff {
		COEFF_OFF, COEFF_2, COEFF_4, COEFF_8, COEFF_16;
	}

	/**
	 * TiI2CMaster object
	 */
	private TiI2CMaster i2cmObj;

	private int i2cSlaveAddress = BME280_I2C_ADDRESS;

	byte[] reg_data = new byte[8];

	long dig_T1, dig_T2, dig_T3;
	long dig_P1, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
	long dig_H1, dig_H3, dig_H2, dig_H4, dig_H5, dig_H6;
	long t_fine;

	double humidity = Double.NaN;
	double temperature = Double.NaN;
	double pressure = Double.NaN;

	/**
	 * Initialization with i2c object
	 * 
	 * @param i2c
	 *            I2C master object for communication
	 */
	public TiBME280(TiI2CMaster i2c) {
		this(i2c, BME280_I2C_ADDRESS);
	}

	public TiBME280(TiI2CMaster i2c, int address) {
		this.i2cmObj = i2c;
		this.i2cSlaveAddress = address;

	}

	/**
	 * Initialize BME280 sensor
	 *
	 * Configure sensor setting and read parameters for calibration Initialize
	 * default parameters. Default configuration: mode: NORAML filter: OFF
	 * oversampling: x4 standby time: 250ms
	 * 
	 * @throws IOException
	 *
	 */
	public void initialize() throws IOException {

		if (this.getDevID() != BME280_CHIP_ID)
			throw new IOException("Invalid chip id.");

		this.softReset();

		Delay.msDelay(100);

		// if chip is still reading calibration, delay
		while (isReadingCalibration())
			Delay.msDelay(100);

		this.getTempPressureCalibration();
		this.getHumidityCalibration();

		this.configSensor(StandByTime.TIME_250_MS, FilterCoeff.COEFF_OFF);
		this.setSensorParameters(Mode.Normal, OverSampling.SAMPLING_4X, OverSampling.SAMPLING_4X,
				OverSampling.SAMPLING_4X);
	}

	/**
	 * parse the temperature and pressure calibration data
	 * 
	 * @throws IOException
	 */
	private void getTempPressureCalibration() throws IOException {

		byte[] cmd = new byte[26];
		// read dig_T regs
		cmd[0] = (byte) TiBME280Register.BME280_CAL_DATA_START_1;
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, cmd, 0, 26);

		dig_T1 = LittleBitConverter.ToUInt16(cmd, 0);
		dig_T2 = LittleBitConverter.ToInt16(cmd, 2);
		dig_T3 = LittleBitConverter.ToInt16(cmd, 4);

		dig_P1 = LittleBitConverter.ToUInt16(cmd, 6);
		dig_P2 = LittleBitConverter.ToInt16(cmd, 8);
		dig_P3 = LittleBitConverter.ToInt16(cmd, 10);
		dig_P4 = LittleBitConverter.ToInt16(cmd, 12);
		dig_P5 = LittleBitConverter.ToInt16(cmd, 14);
		dig_P6 = LittleBitConverter.ToInt16(cmd, 16);
		dig_P7 = LittleBitConverter.ToInt16(cmd, 18);
		dig_P8 = LittleBitConverter.ToInt16(cmd, 20);
		dig_P9 = LittleBitConverter.ToInt16(cmd, 22);

		dig_H1 = cmd[25] & 0Xff;

	}

	/**
	 * parse the humidity calibration data
	 * 
	 * @throws IOException
	 */
	private void getHumidityCalibration() throws IOException {
		byte[] cmd = new byte[7];
		// read dig_T regs
		cmd[0] = (byte) TiBME280Register.BME280_CAL_DATA_START_2;
		this.i2cmObj.write(i2cSlaveAddress, cmd, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, cmd, 0, 7);

		dig_H2 = LittleBitConverter.ToInt16(cmd, 0);
		dig_H3 = cmd[2] & 0xFF;
		dig_H4 = ((cmd[3] & 0xFF) << 4) | (cmd[4] & 0x0f);

		dig_H5 = ((cmd[5] & 0xFF) << 4) | (((cmd[4] & 0xFF) >>> 4) & 0x0f);
		dig_H6 = cmd[6] & 0xff;

	}

	/**
	 * The “id” register contains the chip identification number
	 * 
	 * @return device id (0x60)
	 * @throws IOException
	 */
	public int getDevID() throws IOException {
		reg_data[0] = (byte) TiBME280Register.BME280_CHIP_ID_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		return reg_data[0] & 0xFF;
	}

	private boolean isReadingCalibration() throws IOException {
		reg_data[0] = (byte) TiBME280Register.BME280_STAT_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		int rStatus = reg_data[0] & 0xFF;
		return (rStatus & (1 << 0)) != 0;
	}

	/**
	 * Check if it is busy with measuring temperature/pressure.
	 * 
	 * @return busy or not
	 * @throws IOException
	 */
	public boolean isMeasuring() throws IOException {

		reg_data[0] = (byte) TiBME280Register.BME280_STAT_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		int rStatus = reg_data[0] & 0xFF;

		if ((rStatus & (1 << 3)) > 0) {
			return true;
		}
		return false;
	}

	/**
	 * Soft reset
	 * 
	 * @throws IOException
	 */
	public void softReset() throws IOException {
		reg_data[0] = (byte) TiBME280Register.BME280_RST_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1);
	}

	public void setSensorParameters(Mode mode, OverSampling temperatureOs, OverSampling pressureOs,
			OverSampling humdityOs) throws IOException {

		// Write crtl hum reg first.
		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_HUMIDITY_REG;
		reg_data[1] = (byte) humdityOs.ordinal();
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);

		int ctrlMeas = (temperatureOs.ordinal() << 5) | (pressureOs.ordinal() << 2) | (mode.ordinal());

		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_MEAS_REG;
		reg_data[1] = (byte) ctrlMeas;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);

	}

	public void configSensor(StandByTime standby, FilterCoeff filter) throws IOException {

		int config = (standby.ordinal() << 5) | (filter.ordinal() << 2);

		reg_data[0] = (byte) TiBME280Register.BME280_CONFIG_REG;
		reg_data[1] = (byte) config;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);
	}

	/**
	 * Controls the sensor mode of the device
	 * 
	 * @param m
	 * @throws IOException
	 */
	public void setSensorMode(Mode m) throws IOException {

		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_MEAS_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		int ctrlMeas = reg_data[0] & 0xFF;

		// mask out the bits we care about
		ctrlMeas = ctrlMeas & 0b11111100;

		// Set the magic bits
		ctrlMeas = ctrlMeas | m.ordinal();

		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_MEAS_REG;
		reg_data[1] = (byte) ctrlMeas;

		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);
	}

	/**
	 * Controls oversampling of pressure data
	 * 
	 * @param osr
	 * @throws IOException
	 */
	public void setOversamplingPressure(OverSampling osr) throws IOException {

		int ctrlMeas;
		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_MEAS_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		ctrlMeas = reg_data[0] & 0xFF;

		// change osrs_p which is bits 4,3,2 mask out the bits we care about
		ctrlMeas = ctrlMeas & 0b11100011;
		ctrlMeas = ctrlMeas | (osr.ordinal() << 2); // Set the magic bits

		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_MEAS_REG;
		reg_data[1] = (byte) ctrlMeas;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);

	}

	/**
	 * Controls oversampling of temperature data
	 * 
	 * @param osr
	 * @throws IOException
	 */
	public void setOversamplingTemperature(OverSampling osr) throws IOException {
		int ctrlMeas;
		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_MEAS_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		ctrlMeas = reg_data[0] & 0xFF;

		// change osrs_t which is bits 7,6,5
		ctrlMeas = ctrlMeas & 0b00011111;
		ctrlMeas = ctrlMeas | (osr.ordinal() << 5); // Set the magic bits

		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_MEAS_REG;
		reg_data[1] = (byte) ctrlMeas;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);
	}

	/**
	 * Controls oversampling of humidity datar
	 * 
	 * @param osr
	 * @throws IOException
	 */
	public void setOversamplingHumidity(OverSampling osr) throws IOException {
		// ctrl_hum
		reg_data[0] = (byte) TiBME280Register.BME280_CTRL_HUMIDITY_REG;

		// Humidity oversampling x1
		reg_data[1] = (byte) osr.ordinal();
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);
	}

	/**
	 * Controls inactive duration standby time in normal mode
	 * 
	 * @param sbt
	 * @throws IOException
	 */
	public void setStandbyTime(StandByTime sbt) throws IOException {
		int conf;
		reg_data[0] = (byte) TiBME280Register.BME280_CONFIG_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		conf = reg_data[0] & 0xFF;

		conf = conf & 0b00011111; //
		conf = conf | (sbt.ordinal() << 5); // Set the magic bits

		reg_data[0] = (byte) TiBME280Register.BME280_CONFIG_REG;
		reg_data[1] = (byte) conf;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);

	}

	/**
	 * Controls the time constant of the IIR filter
	 * 
	 * @param fc
	 * @throws IOException
	 */
	public void setFilterCoefficient(FilterCoeff fc) throws IOException {
		int conf;
		reg_data[0] = (byte) TiBME280Register.BME280_CONFIG_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 1);

		conf = reg_data[0] & 0xFF;

		// change osrs_p which is bits 4,3,2
		conf = conf & 0b11100011; // mask out the bits we care about
		conf = conf | (fc.ordinal() << 2); // Set the magic bits

		reg_data[0] = (byte) TiBME280Register.BME280_CONFIG_REG;
		reg_data[1] = (byte) conf;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 2);

	}

	public double getTemperature() {
		return this.temperature;
	}

	public double getHumidity() {
		return this.humidity;
	}

	public double getPressure() {
		return this.pressure;
	}

	/**
	 * Read register data and compensate the temperature, humidity and pressure
	 * 
	 * @throws IOException
	 */
	public void measure() throws IOException {

		reg_data[0] = (byte) TiBME280Register.BME280_PRESSURE_MSB_REG;
		this.i2cmObj.write(i2cSlaveAddress, reg_data, 0, 1, true);
		this.i2cmObj.read(i2cSlaveAddress, reg_data, 0, 8);

		long data_xlsb;
		long data_lsb;
		long data_msb;

		/* Store the parsed register values for pressure data */
		data_msb = ((long) (reg_data[0] & 0xFF)) << 12;
		data_lsb = ((long) (reg_data[1] & 0xFF)) << 4;
		data_xlsb = ((long) (reg_data[2] & 0xFF)) >>> 4;

		long pressure_raw = data_msb | data_lsb | data_xlsb;

		/* Store the parsed register values for temperature data */
		data_msb = ((long) (reg_data[3] & 0xFF)) << 12;
		data_lsb = ((long) (reg_data[4] & 0xFF)) << 4;
		data_xlsb = ((long) (reg_data[5] & 0xFF)) >>> 4;

		long temperature_raw = data_msb | data_lsb | data_xlsb;

		/* Store the parsed register values for humidity data */
		data_lsb = ((long) (reg_data[6] & 0xFF)) << 8;
		data_msb = ((long) (reg_data[7] & 0xFF));

		long humidity_raw = data_msb | data_lsb;

		this.temperature = this.compensate_temperature(temperature_raw);
		this.humidity = this.compensate_humidity(humidity_raw);
		this.pressure = this.compensate_pressure(pressure_raw);
	}

	/**
	 * compensate the raw pressure data and return the compensated pressure data
	 * in double data type.
	 * 
	 * @param pressure_raw
	 * @return pressure in Pa as double. Output value of “96386.2” equals
	 *         96386.2 Pa = 963.862 hPa
	 */
	private double compensate_pressure(long pressure_raw) {
		double var1;
		double var2;
		double var3;
		double pressure;
		double pressure_min = 30000.0;
		double pressure_max = 110000.0;

		if (pressure_raw == 0x80000) {
			return Double.NaN;
		}

		var1 = ((double) t_fine / 2.0) - 64000.0;
		var2 = var1 * var1 * ((double) dig_P6) / 32768.0;
		var2 = var2 + var1 * ((double) dig_P5) * 2.0;
		var2 = (var2 / 4.0) + (((double) dig_P4) * 65536.0);
		var3 = ((double) dig_P3) * var1 * var1 / 524288.0;
		var1 = (var3 + ((double) dig_P2) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0) * ((double) dig_P1);

		/* avoid exception caused by division by zero */
		if (var1 > 0) {
			pressure = 1048576.0 - (double) pressure_raw;
			pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
			var1 = ((double) dig_P9) * pressure * pressure / 2147483648.0;
			var2 = pressure * ((double) dig_P8) / 32768.0;
			pressure = pressure + (var1 + var2 + ((double) dig_P7)) / 16.0;

			if (pressure < pressure_min)
				pressure = pressure_min;

			else if (pressure > pressure_max)
				pressure = pressure_max;

		} else { /* Invalid case */
			pressure = pressure_min;
		}

		return pressure;
	}

	/**
	 * compensate the raw humidity data and return the compensated humidity data
	 * in double data type.
	 * 
	 * @param humidity_raw
	 * @return humidity in %rH as as double. Output value of “46.332” represents
	 *         46.332 %rH
	 */
	private double compensate_humidity(long humidity_raw) {
		double humidity;
		double humidity_min = 0.0;
		double humidity_max = 100.0;
		double var1;
		double var2;
		double var3;
		double var4;
		double var5;
		double var6;

		var1 = ((double) t_fine) - 76800.0;
		var2 = (((double) dig_H4) * 64.0 + (((double) dig_H5) / 16384.0) * var1);
		var3 = humidity_raw - var2;
		var4 = ((double) dig_H2) / 65536.0;
		var5 = (1.0 + (((double) dig_H3) / 67108864.0) * var1);
		var6 = 1.0 + (((double) dig_H6) / 67108864.0) * var1 * var5;
		var6 = var3 * var4 * (var5 * var6);
		humidity = var6 * (1.0 - ((double) dig_H1) * var6 / 524288.0);

		if (humidity > humidity_max)
			humidity = humidity_max;
		else if (humidity < humidity_min)
			humidity = humidity_min;

		return humidity;
	}

	/**
	 * compensate the raw temperature data and return the compensated
	 * temperature data in integer data type
	 * 
	 * @param temperature_raw
	 * @return temperature in DegC, double precision. Output value of “51.23”
	 *         equals 51.23 DegC. t_fine carries fine temperature as global
	 *         value
	 */
	private double compensate_temperature(long temperature_raw) {

		double var1;
		double var2;
		double temperature;
		double temperature_min = -40;
		double temperature_max = 85;

		var1 = ((double) temperature_raw) / 16384.0 - ((double) dig_T1) / 1024.0;
		var1 = var1 * ((double) dig_T2);
		var2 = (((double) temperature_raw) / 131072.0 - ((double) dig_T1) / 8192.0);
		var2 = (var2 * var2) * ((double) dig_T3);
		t_fine = (long) (var1 + var2);
		temperature = (var1 + var2) / 5120.0;

		if (temperature < temperature_min)
			temperature = temperature_min;
		else if (temperature > temperature_max)
			temperature = temperature_max;

		return temperature;
	}

}
