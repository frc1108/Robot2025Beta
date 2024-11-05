// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HendersonConstants;

@Logged
public class HendersonFeeder extends SubsystemBase {
  private final CANSparkMax m_leftMotor = new CANSparkMax(HendersonConstants.kLeftFeederMotorCanId,MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(HendersonConstants.kRightFeederMotorCanId,MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkLimitSwitch m_limitSwitch;

  /** Creates a new HendersonFeeder. */
  public HendersonFeeder() {
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();

    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotor.setIdleMode(IdleMode.kBrake);

    m_rightMotor.follow(m_leftMotor, true);
    m_encoder = m_leftMotor.getEncoder();

    m_limitSwitch = m_leftMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();

  }
  public void set(double speed){
    m_leftMotor.set(speed);
  }

  @Logged(name = "Note Sensor Triggered")
  public boolean getBeamBreak(){
    return m_leftMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public void enableLimitSwitches(){
    m_limitSwitch.enableLimitSwitch(true);
  }

  public void disableLimitSwitches(){
    m_limitSwitch.enableLimitSwitch(false);
    if (m_limitSwitch.isLimitSwitchEnabled()) {
          m_limitSwitch.enableLimitSwitch(false);
    }
  }

  // public Command run(){
  //   return Commands.runOnce(()->set(0.5));
  // }
  // public Command runReverse(){
  //   return Commands.runOnce(()->set(-0.35));
  // }
  public Command stop() {
    return Commands.runOnce(()->set(0));
  }

  public Command runStopCommand(){
    return Commands.sequence(runOnce(()->set(0.8)),
             Commands.race(Commands.waitSeconds(5),
                           Commands.waitUntil(this::getBeamBreak)),
             runOnce(()->set(0)).withName("Beam Feeder"));
  }
}
