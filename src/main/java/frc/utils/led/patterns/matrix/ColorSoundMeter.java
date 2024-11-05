// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils.led.patterns.matrix;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.led.TrobotAddressableLEDPattern;

public class ColorSoundMeter implements TrobotAddressableLEDPattern{
	Color m_color;
	DoubleSupplier m_soundMeterDouble;
	public ColorSoundMeter(DoubleSupplier soundMeterDouble, Color onColor){
		super();
		m_color = onColor;
		m_soundMeterDouble = soundMeterDouble;
	}

	@Override
	public void setLEDs(AddressableLEDBuffer buffer) {
		int row_count = 8;
		var On = Math.round(m_soundMeterDouble.getAsDouble()  * buffer.getLength() / row_count )* row_count;
	
		for (int index = 0; index < buffer.getLength(); index++){
			if (index < On) {
				buffer.setLED(index, m_color);
			} else {
				buffer.setLED(index, Color.kBlack);
			}}
	}
	
	public boolean isAnimated(){
		return true;
	}
}