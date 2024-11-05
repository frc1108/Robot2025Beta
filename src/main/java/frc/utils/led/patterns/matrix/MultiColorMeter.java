// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.led.patterns.matrix;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.led.TrobotAddressableLEDPattern;

public class MultiColorMeter implements TrobotAddressableLEDPattern{
	Color m_color;
	DoubleSupplier m_meterLevel;
	public MultiColorMeter(DoubleSupplier meterLevel){
		super();
		m_meterLevel = meterLevel;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int currentHue;
		int row_count = 8;
		int length = buffer.getLength();
		var On = Math.round(m_meterLevel.getAsDouble()  * length / row_count )* row_count;

		for (int index = 0; index < buffer.getLength(); index++){
			if (index < On) {
				currentHue = 300-(((index * 2160 / row_count) / buffer.getLength()) % 300);
				buffer.setHSV(index, currentHue, 255, 128);
			} else {
				buffer.setLED(index, Color.kBlack);
			}}
	}

	public boolean isAnimated(){
		return true;
	}
}