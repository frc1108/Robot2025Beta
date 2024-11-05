// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UnderrollerConstants;


@Logged 
public class Underroller extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax m_underrollerFront = new CANSparkMax(UnderrollerConstants.kFrontCanId, MotorType.kBrushless);
  private final CANSparkMax m_underrollerRear = new CANSparkMax(UnderrollerConstants.kRearCanId, MotorType.kBrushless);
  public Underroller() {
    m_underrollerFront.setIdleMode(IdleMode.kBrake);
    m_underrollerRear.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

public void setUnderrollerspeed (double speed ){
  m_underrollerFront.set(speed );
  m_underrollerRear.set(speed );
}


  public Command runUnderroller(){
    return Commands.startEnd(()->setUnderrollerspeed(0.6),()->setUnderrollerspeed(0));
  }
  public Command reverseUnderroller(){
    return Commands.startEnd(()->setUnderrollerspeed(-0.35),()->setUnderrollerspeed(0));
  }
}
