package tijos.framework.sensor.bme280;

import java.io.IOException;

import tijos.framework.devicecenter.TiI2CMaster;
import tijos.framework.util.Delay;

public class TiBME280Sample { 
 
	public static void main(String[] args) {

		try {
			/*
			 * TiI2CMaster port 0
			 */
			int i2cPort0 = 0;

			/*
			 * Open the I2C port 
			 */
			TiI2CMaster i2c0 = TiI2CMaster.open(i2cPort0);

			TiBME280 bme280 = new TiBME280(i2c0);

			bme280.initialize();	
			
			int num = 1000;
			while (num -- > 0) {
				try {

					bme280.measure();

					double temperature = bme280.getTemperature();
					double pressure = bme280.getPressure() / 100;
					double humidity = bme280.getHumidity();

					System.out.println("pressure = " + pressure + " temperature = " + temperature + " humidity = " + humidity);

					Delay.msDelay(2000);
					
				} catch (Exception ex) {

					ex.printStackTrace();
				}

			}
		} catch (IOException ie) {
			ie.printStackTrace();
		}


	}

}
