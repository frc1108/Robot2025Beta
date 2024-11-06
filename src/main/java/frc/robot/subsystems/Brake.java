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

import edu.wpi.first.epilogue.Logged;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Brake extends SubsystemBase {
  private final SparkMax m_motor = new SparkMax(51, MotorType.kBrushed);
  private static final Time MOTOR_RUN_TIME = Seconds.of(0.3);

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
    return Commands.startEnd(
      ()->set(0.75),
      ()->set(0),this)
      .withTimeout(MOTOR_RUN_TIME.magnitude());
  }

  public Command unbrake() {
    return Commands.startEnd(
      ()->set(-0.75),
      ()->set(0),this)
      .withTimeout(MOTOR_RUN_TIME.magnitude());
  }
}
