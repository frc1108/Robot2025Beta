// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Brake extends SubsystemBase {
  /** Creates a new Brake. */
  private final SparkMax m_motor = new SparkMax(51, MotorType.kBrushed);
  public Brake() {
    var config = new SparkMaxConfig();
    config.inverted(true)
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(30);
    m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void set(double speed) {
    m_motor.set(speed);
  }

  public Command brake() {
    return Commands.startEnd(()->set(0.75),()->set(0),this).withTimeout(0.3);
  }

  public Command unbrake() {
    return Commands.startEnd(()->set(-0.75),()->set(0),this).withTimeout(0.3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
