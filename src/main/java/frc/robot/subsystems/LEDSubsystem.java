// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.ListIterator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.utils.led.TrobotAddressableLED;
import frc.utils.led.TrobotAddressableLEDPattern;
import frc.utils.led.patterns.matrix.ColorSoundMeter;
import frc.utils.led.patterns.matrix.MultiColorMeter;
import frc.utils.led.patterns.strip.BlinkingPattern;
import frc.utils.led.patterns.strip.RainbowPattern;
import frc.utils.led.patterns.strip.SolidColorPattern;
import edu.wpi.first.wpilibj.LEDPattern;

public class LEDSubsystem extends SubsystemBase {
  private TrobotAddressableLED m_led = new TrobotAddressableLED(LedConstants.kLedPWMPort,
                                       LedConstants.kLedCount);

  
  private TrobotAddressableLEDPattern m_bluePattern = new SolidColorPattern(Color.kBlue);
  private TrobotAddressableLEDPattern m_blackPattern = new SolidColorPattern(Color.kBlack);
	private TrobotAddressableLEDPattern m_redPattern = new SolidColorPattern(Color.kRed);
	private TrobotAddressableLEDPattern m_purplePattern = new SolidColorPattern(Color.kPurple);
	private TrobotAddressableLEDPattern m_yellowPattern = new SolidColorPattern(Color.kYellow);
	private TrobotAddressableLEDPattern m_orangePattern = new SolidColorPattern(Color.kOrange);
	private TrobotAddressableLEDPattern m_flashPattern = new BlinkingPattern(Color.kWhite,0.1);
	private TrobotAddressableLEDPattern m_disabledPattern = new RainbowPattern();
	private TrobotAddressableLEDPattern m_rainbowMeter; 
	private TrobotAddressableLEDPattern m_blueSoundMeter; 
	private TrobotAddressableLEDPattern m_redSoundMeter;
  private final LEDPattern m_testing = LEDPattern.solid(Color.kBlue);  
  private TrobotAddressableLEDPattern m_currentPattern;
  private List<TrobotAddressableLEDPattern> m_patternList;
  private ListIterator<TrobotAddressableLEDPattern> m_patternIterator;
  private AnalogInput m_mic = new AnalogInput(LedConstants.kVolumeSensorPort);
  private final AnalogInputSim m_micSim;
  private LinearFilter m_micFilter = LinearFilter.movingAverage(20);
  //private MedianFilter m_micFilter = new MedianFilter(20);
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_rainbowMeter = new MultiColorMeter(()->this.getMicOutput());
    m_blueSoundMeter = new ColorSoundMeter(()->0.5*this.getMicOutput(),Color.kBlue);
    m_redSoundMeter = new ColorSoundMeter(()->0.5*this.getMicOutput(),Color.kRed);
    // m_rainbowMeter = new MultiColorMeter(()->m_controller.getRightTriggerAxis());
    // m_blueSoundMeter = new ColorSoundMeter(()->m_controller.getRightTriggerAxis(),Color.kBlue);
    // m_redSoundMeter = new ColorSoundMeter(()->m_controller.getRightTriggerAxis(),Color.kRed);

    m_patternList = new ArrayList<TrobotAddressableLEDPattern>
                        (Arrays.asList(m_redPattern,m_bluePattern,m_blackPattern,m_flashPattern,
                                       m_yellowPattern, m_orangePattern, m_purplePattern,
                                       m_disabledPattern,m_rainbowMeter,m_blueSoundMeter,m_redSoundMeter));


    m_patternIterator = m_patternList.listIterator();
    
    m_currentPattern =  m_redPattern;

    AnalogInput.setGlobalSampleRate(50000);
    m_micSim = new AnalogInputSim(m_mic);


  }


  public void nextPattern() {
       if (!m_patternIterator.hasNext()) {
        m_patternIterator = m_patternList.listIterator();
      }
      m_currentPattern = m_patternIterator.next();
  }

  public void flash(){
    setPattern(m_flashPattern);
  }

  public void off() {
    setPattern(m_blackPattern);
  }

  public void yellow() {
    setPattern(m_yellowPattern);
  }

  public void orange() {
    setPattern(m_yellowPattern);
  }

  public void hasNote() {
    setPattern(m_yellowPattern);
  }

  public void rainbow() {
    setPattern(m_disabledPattern);
  }

  public void startCrowdMeter(Alliance alliance) {
    if (alliance == Alliance.Blue) {
      setPattern(m_blueSoundMeter);
    } else {
      setPattern(m_redSoundMeter);
    }
  }
//   public void setConePattern() {
//     setPattern(m_yellowPattern);
//   }

//   public void setCubePattern() {
//     setPattern(m_purplePattern);
//   }

//   public void raiseTheNoise(double level) {
//     setPattern(new SolidColorPattern(Color.kBlue));
//   }

  public void setPattern(TrobotAddressableLEDPattern pattern) {
    m_currentPattern = pattern;
    var patternExists = m_patternList.contains(pattern);
    if (patternExists) {
        m_patternIterator = m_patternList.listIterator(
                                      m_patternList.indexOf(pattern));
    }
  }

  @Override
  public void periodic() {
    m_led.setPattern(m_currentPattern);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Mic Voltage",this.getMicVoltage());
    SmartDashboard.putNumber("Mic Avg Voltage",this.getMicAverageVoltage());
    SmartDashboard.putNumber("Mic Filtered",this.getMicFiltered());
    SmartDashboard.putNumber("Mic Output",this.getMicOutput());
  }

   public double getMicVoltage() {
    if (RobotBase.isReal()) {
      return Math.pow(m_mic.getVoltage()-m_mic.getAverageVoltage(),2);
    } else {
      return Math.pow(m_micSim.getVoltage()-2.5,2);
    }
  }

  public double getMicAverageVoltage() {
    return m_mic.getAverageVoltage();
  }

  public double getMicFiltered() {
    try {
      return m_micFilter.calculate(getMicVoltage());
    } catch (NullPointerException e) {
      System.out.println(e);
      return 0;
    }
  }

  public double getMicOutput() {
    return getMicFiltered();
    //return MathUtil.clamp(getMicFiltered(),0,1);
  } 
}
