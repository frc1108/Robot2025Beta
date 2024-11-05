// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HendersonConstants;

public class HendersonLauncher extends SubsystemBase implements Logged{
  private final CANSparkFlex m_leftMotor = new CANSparkFlex(HendersonConstants.kLeftLauncherMotorCanId,MotorType.kBrushless);
  private final CANSparkFlex m_rightMotor = new CANSparkFlex(HendersonConstants.kRightLauncherMotorCanId,MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkPIDController m_pidController;
  private double m_goal = 0;
  private boolean m_enabled = false;

  /** Creates a new HendersonFeeder. */
  public HendersonLauncher() {
    m_leftMotor.restoreFactoryDefaults();
    m_rightMotor.restoreFactoryDefaults();
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.follow(m_leftMotor, true);
    
    m_encoder = m_leftMotor.getEncoder();
    m_pidController = m_leftMotor.getPIDController();
    m_pidController.setP(HendersonConstants.kLauncherP);
    m_pidController.setI(HendersonConstants.kLauncherI);
    m_pidController.setD(HendersonConstants.kLauncherD);
    m_pidController.setFF(HendersonConstants.kLauncherFF);
    m_pidController.setOutputRange(HendersonConstants.kLauncherOutputMin,
                                  HendersonConstants.kLauncherOutputMax);
        
    m_leftMotor.burnFlash();
    m_rightMotor.burnFlash();
  }

  @Override
  public void periodic() {
     if (m_enabled) {
      m_pidController.setReference(m_goal, ControlType.kVelocity);
     }
     SmartDashboard.putNumber("Launcher Speed",m_encoder.getVelocity());

    // This method will be called once per scheduler run
  }

  public void set(double speed) {
    disablePid();
    m_leftMotor.set(speed);
  }

  public Command run() {
    return Commands.runOnce(()->set(0.5));
  }

  public Command runReverse() {
    return Commands.runOnce(()->set(-0.35));
  }

  public Command stop() {
    disablePid();
    return Commands.runOnce(()->set(0));
  }

  public Command idle() {
    return Commands.runOnce(()->setRPM(HendersonConstants.kLauncherIdleRpm));
  }

  private void setRPM(double goal) {
    enablePid();
    m_goal = MathUtil.clamp(goal, 0, HendersonConstants.kMaxLauncherRpm);
  }

  public void enablePid() {
    m_enabled = true;
  }

  public void disablePid() {
    m_enabled = false;
    m_goal = 0;
  }

  @Log.NT(key = "Shooter Vel RPM")
  public double getVelocity() {
    return m_encoder.getVelocity();
  }

  @Log.NT(key = "Shooter Goal RPM")
  public double getGoal() {
    return m_goal;
  }
}
