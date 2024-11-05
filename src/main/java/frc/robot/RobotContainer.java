// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HendersonConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.UnderrollerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Brake;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Underroller;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.HendersonFeeder;
import frc.robot.subsystems.HendersonLauncher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

@Logged
public class RobotContainer{
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Underroller m_underroller = new Underroller();
  private final Arm m_arm = new Arm();
  private final HendersonFeeder m_feeder = new HendersonFeeder();
  private final HendersonLauncher m_launcher  = new HendersonLauncher();
  private final LEDSubsystem m_led = new LEDSubsystem();
  private final Brake m_brake = new Brake();
  private Vision m_vision;


  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final Field2d m_path;
  @Logged(name = "Auto Selector") private final SendableChooser<Command> m_autoChooser;
  private int m_invertDriveAlliance = -1;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
  

    m_path = new Field2d();
    
    // Robot configs
    configureButtonBindings();

    // 
    configureNamedCommands();
    m_autoChooser = AutoBuilder.buildAutoChooser();

    setupPathPlannerLog();
    try {
      m_vision = new Vision(m_robotDrive::visionPose, m_robotDrive);
    }
     catch(IOException e) {
     DriverStation.reportWarning("Unable to initialize vision", e.getStackTrace());
    }

    // Default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                m_invertDriveAlliance*MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));

      //           m_arm.setDefaultCommand(
      // new RunCommand(
      //   () -> m_arm.set(-ArmConstants.kMaxArmSpeed*
      //     MathUtil.applyDeadband(m_operatorController.getRightY(),
      //     ArmConstants.kArmDeadband)),m_arm));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //**** TRIGGERS ****/
    //RobotModeTriggers.autonomous().onTrue(Commands.runOnce(()->m_feeder.enableLimitSwitches()));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(()->m_feeder.disableLimitSwitches()));
    new Trigger(()->!m_feeder.getBeamBreak()).and(()->!this.intakeNote().isScheduled()).onTrue(Commands.runOnce(()->m_led.off()));
    new Trigger(()->m_feeder.getBeamBreak()).and(()->!this.intakeNote().isScheduled()).onTrue(Commands.runOnce(()->m_led.orange()));


    // RobotModeTriggers.disabled().onTrue(Commands.sequence(Commands.runOnce(()->m_led.rainbow()),
    //                                                       Commands.waitSeconds(6),
    //                                                       Commands.runOnce(()->m_arm.setIdle(IdleMode.kCoast))));
    m_operatorController.axisGreaterThan(0, 0.5).debounce(0.2).onTrue(Commands.print("Axis Trigger"));                                                      
    //**** DRIVER CONTROLS ****/
    //m_driverController.a().onTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.x().onTrue(Commands.runOnce(() -> m_robotDrive.startVisionPose()));
    m_driverController.y().onTrue(Commands.parallel(
                                    Commands.runOnce(() -> m_robotDrive.stopVisionPose()),
                                    Commands.runOnce(()->m_robotDrive.zeroHeading())
                                  ));
                                  //.andThen(Commands.runOnce(()->m_robotDrive.resetOdometry(m_robotDrive.getPose()))));
    m_driverController.b().whileTrue(Commands.run(() -> m_robotDrive.setX(),m_robotDrive));
    m_driverController.povLeft().onTrue(Commands.runOnce(m_led::nextPattern,m_led));
    m_operatorController.back().onTrue(m_brake.brake());
    m_driverController.back().onTrue(m_brake.unbrake());

    //**** OPERATOR CONTROLS ****
    m_operatorController.a().whileTrue(Commands.runEnd(
                                         ()->m_feeder.set(HendersonConstants.kFeederBackSpeed),
                                         ()->m_feeder.set(0),m_feeder));
    m_operatorController.b().whileTrue(Commands.runEnd(
                                         ()->m_feeder.set(HendersonConstants.kFeederFrontSpeed),
                                         ()->m_feeder.set(0),m_feeder));
    m_operatorController.x().whileTrue(Commands.runEnd(
                                         ()->m_launcher.set(HendersonConstants.kLauncherFrontSpeed),
                                         ()->m_launcher.set(0),m_launcher));
    m_operatorController.y().whileTrue(Commands.runEnd(
                                         ()->m_launcher.set(HendersonConstants.kLauncherBackSpeed),
                                         ()->m_launcher.set(0),m_launcher));
    
    // m_operatorController.axisGreaterThan(0,0.75).onTrue(shoot());
    // m_operatorController.axisLessThan(0,-0.75).onTrue(shootBackwards());
    // m_operatorController.axisGreaterThan(1,0.75).onTrue(intakeNote());
    // m_operatorController.axisLessThan(1,-0.75).onTrue(centeringNote());

    m_operatorController.start().onTrue(Commands.runOnce(()->m_arm.enableClimb()));
    m_operatorController.back().onTrue(Commands.runOnce(()->m_arm.disableClimb()));
    
    m_operatorController.leftBumper().whileTrue(m_underroller.runUnderroller().withName("Intaking"));
    m_operatorController.rightBumper().whileTrue(m_underroller.reverseUnderroller().withName("Outtaking"));
    
    m_operatorController.povDown().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads));
    m_operatorController.povUp().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmShootingAngleRads));
    m_operatorController.povRight().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmFarShootingAngleRads));
    m_operatorController.povLeft().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmDownRads));
    //m_operatorController.povLeft().onTrue(m_arm.setArmGoalCommand(ArmConstants.kArmDownRads));

    m_operatorController.axisGreaterThan(5,-0.75).onTrue(
      m_arm.setArmGoalCommand(ArmConstants.kArmFarShootingAngleRads+1*ArmConstants.kArmShootingStepsRads));
    m_operatorController.axisGreaterThan(4,0.75).onTrue(
      m_arm.setArmGoalCommand(ArmConstants.kArmFarShootingAngleRads+2*ArmConstants.kArmShootingStepsRads));
    m_operatorController.axisGreaterThan(5,0.75).onTrue(
      m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads+1*ArmConstants.kArmPickupStepsRads));
    m_operatorController.axisGreaterThan(4,-0.75).onTrue(
      m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads+2*ArmConstants.kArmPickupStepsRads));
    m_operatorController.axisGreaterThan(2,0.75).onTrue(
      m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads));
    m_operatorController.axisGreaterThan(3,0.75).onTrue(
      m_arm.setArmGoalCommand(ArmConstants.kArmFeederAngle));

    m_driverController.axisGreaterThan(3, 0.75).onTrue(Commands.runOnce(()->m_feeder.disableLimitSwitches()));
      }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  private void configureNamedCommands() {
      NamedCommands.registerCommand("LaunchNote", shoot());
      NamedCommands.registerCommand("IntakeNote", intakeNote());
      NamedCommands.registerCommand("AmpShot", AmpShot());
      NamedCommands.registerCommand("Stop", Stop());
      NamedCommands.registerCommand("ShootBackwards", shootBackwards());
      NamedCommands.registerCommand("CenteringNote", centeringNote());
      NamedCommands.registerCommand("AllWheelsForward", m_robotDrive.setAllWheelsForward());
      NamedCommands.registerCommand("AllWheelsRight", m_robotDrive.setAllWheelsRight());
    }

  public void configureWithAlliance(Alliance alliance) {
    m_led.startCrowdMeter(alliance);
    m_invertDriveAlliance = (alliance == Alliance.Blue)?-1:1;
  }
   
  public Command intakeNote() {
    return 
      Commands.sequence(
        m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads),
        Commands.waitSeconds(0.1),
        Commands.parallel(
          Commands.runOnce(()->m_launcher.set(HendersonConstants.kIntakeLauncherSpeed),m_launcher),
          Commands.runOnce(()->m_feeder.set(HendersonConstants.kIntakeFeederSpeed),m_feeder),
          Commands.runOnce(()->m_underroller.setUnderrollerspeed(UnderrollerConstants.kUnderrollerIntakeSpeed),m_underroller),
          Commands.runOnce(()->m_led.yellow()),
          Commands.runOnce(()->m_feeder.enableLimitSwitches())
        ),
        Commands.race(Commands.waitUntil(()->m_feeder.getBeamBreak()),Commands.waitSeconds(5))
        .finallyDo(()->
          Commands.parallel(
            Commands.runOnce(()->m_launcher.set(0),m_launcher),
            Commands.runOnce(()->m_feeder.set(0),m_feeder),
            Commands.runOnce(()->m_underroller.setUnderrollerspeed(0),m_underroller),
            Commands.runOnce(()->m_feeder.disableLimitSwitches()),
            Commands.runOnce(()->m_led.off())
          )
        )
      );
  }

  public Command shoot() {
    return Commands.sequence(
        m_arm.setArmGoalCommand(ArmConstants.kArmShootingAngleRads),
        Commands.waitSeconds(0.5),
        Commands.runOnce(()->m_launcher.set(-0.8)),
        Commands.waitSeconds(0.75),
        Commands.runOnce(()->m_feeder.set(-0.6)),
        Commands.waitSeconds(0.2),
        m_feeder.stop(),
        Commands.runOnce(()->m_launcher.set(0))
     );
  }

  public Command AmpShot() {
     return Commands.sequence(
         m_arm.setArmGoalCommand(ArmConstants.kArmShootingAngleRads),
         Commands.waitSeconds(1.75),
         Commands.runOnce(()->m_feeder.set(1.0)),
         Commands.waitSeconds(1.0),
         Commands.runOnce(()->m_feeder.set(0.0)),
         m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads)
      );
   }

  public Command Stop() {
     return Commands.sequence(
         Commands.runOnce(()->m_launcher.set(0)),
         Commands.runOnce(()->m_feeder.set(0)),
         Commands.runOnce(()->m_underroller.setUnderrollerspeed(0),m_underroller)
      );
   }
  
    public Command shootBackwards() {
    return Commands.sequence(
      Commands.runOnce(()->m_feeder.disableLimitSwitches()),
      m_arm.setArmGoalCommand(ArmConstants.kArmPickupAngleRads),
      Commands.waitSeconds(0.3),
      Commands.runOnce(()->m_feeder.set(1)),
      Commands.waitSeconds(0.3),
      Commands.runOnce(()->m_launcher.set(0.8)),
      Commands.waitSeconds(0.75)).finallyDo(()->
        Commands.parallel(
                        Commands.runOnce(()->m_launcher.set(0.0)),
                        Commands.runOnce(()->m_feeder.set(0.0)),
                        Commands.runOnce(()->m_feeder.enableLimitSwitches())));
  }

  public Command centeringNote() {
    return Commands.sequence(
      Commands.parallel(Commands.runOnce(()->m_launcher.set(0.1)),
                        Commands.runOnce(()->m_feeder.set(-0.5))),
       Commands.waitSeconds(0.05)).finallyDo(()->
        Commands.parallel(
                        Commands.runOnce(()->m_launcher.set(0.0)),
                        Commands.runOnce(()->m_feeder.set(0.0))));
  }

  private void setupPathPlannerLog() {
    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        m_path.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      m_path.getObject("target pose").setPose(pose);
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      // Do whatever you want with the poses here
      m_path.getObject("path").setPoses(poses);
    });
  }
  
}