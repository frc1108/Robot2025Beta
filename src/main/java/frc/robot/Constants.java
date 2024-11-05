// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.76; //5.5
    public static final double kMaxAngularSpeed = 2*Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.5; // radians per second 1.2
    public static final double kMagnitudeSlewRate = 2.0; // percent per second (1 = 100%) 1.8
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    // Drive Module constants
    public static final double kFrontLeftDrivingkP = 0.005; //0.125
    public static final double kFrontLeftDrivingkS = 0.05; //0.125
    public static final double kFrontLeftDrivingkV = 2.5; //2.21
    public static final double kFrontLeftDrivingkA = 0.24; //0.37 //0.095

    public static final double kFrontRightDrivingkP = 0.005; //0.125
    public static final double kFrontRightDrivingkS = 0.05; //0.125
    public static final double kFrontRightDrivingkV = 2.18; //2.21
    public static final double kFrontRightDrivingkA = 0.25; //0.37

    public static final double kRearLeftDrivingkP = 0.005;
    public static final double kRearLeftDrivingkS = 0.05;
    public static final double kRearLeftDrivingkV = 2.21;
    public static final double kRearLeftDrivingkA = 0.27;

    public static final double kRearRightDrivingkP = 0.005;
    public static final double kRearRightDrivingkS = 0.05;
    public static final double kRearRightDrivingkV = 2.5;
    public static final double kRearRightDrivingkA = 0.28;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = VortexMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.072; //72; // 0.072; // 0.0762 (Caliper and squeeze 3/28 3D printed -- Max 0.0753)
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    // TEST Values turnoff PID
    // public static final double kDrivingP = 0.04; 
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 0; //1.1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 80; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final PIDConstants kTranslationPid = new PIDConstants(5,0,0);
    public static final PIDConstants kRotationPid = new PIDConstants(5,0,0);
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6704;
  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }


  public static final class LedConstants {
    public static final int kLedPWMPort = 0;
    public static final int kLedCount = 256;
    public static final int kVolumeSensorPort = 3;
  }

  public static final class UnderrollerConstants {
    public static final int kFrontCanId = 21;
    public static final int kRearCanId = 22;
    public static final double kUnderrollerIntakeSpeed = 0.8;
  }
  
  public static final class ArmConstants {
    public static final int kLeftArmMotorCanId = 31;
    public static final int kRightArmMotorCanId = 32;

    public static final double kSportGearRatio = 36.0;
    public static final double kSportPinionTeeth = 10;
    public static final double kArmSprocketTeeth = 36;
    public static final double kArmGearRatio = 129.6;

    // SysID values (in radians and radians/sec)
    public static final double kSVolts = 1.8; //0.11356;
    public static final double kGVolts = 0.5; //0.29175;
    public static final double kVVoltSecondPerRad = 7; //3.65; //1.5928;
    public static final double kAVoltSecondSquaredPerRad = 0.02;//0.030171;

    // Set the arm speed and acceleration
    public static final double kMaxVelocityRadPerSecond = 1.5;
    public static final double kMaxAccelerationRadPerSecSquared = 2;

    public static final double kArmOffsetRads = Units.degreesToRadians(-45); //Starting angle
    public static final double kArmMaxRads = Units.degreesToRadians(90); //Ending angle

    public static final double kArmEncoderPositionFactor = ((2 * Math.PI) / kArmGearRatio); // radians
    public static final double kArmEncoderVelocityFactor = ((2 * Math.PI) / kArmGearRatio) / 60.0; // radians per second

    public static final double kArmEncoderPositionPIDMinInput = kArmOffsetRads; // radians
    public static final double kArmEncoderPositionPIDMaxInput = kArmMaxRads; 

    public static final int kArmMotorCurrentLimit = 50; // amps
    
    public static final double kP = 4; //10000x
    public static final double kI = 0;
    public static final double kD = 0.5;
    public static final double kDVel = 0;
    public static final double kFF = 0;
    public static final double kMinOutput = -1;
    public static final double kMaxOutput = 1;
    // public static final double kMaxPositionErrorRad = 0.7854;
    // public static final double kMaxVelocityErrorRadPerSec = 1.8656;
    // public static final double kControlEffortVolts = 7;
    public static final int kSlotDefault = 0;
    public static final int kSlotClimb = 1;

    public static final double kPClimb = 8; //10000x
    public static final double kIClimb = 0;
    public static final double kDClimb = 0.5;
    public static final double kFFClimb = 0;
    public static final double kMinOutputClimb = -1;
    public static final double kMaxOutputClimb = 1;

    public static final double kMaxArmSpeedRpm = 
      NeoMotorConstants.kFreeSpeedRpm / kArmGearRatio ;
    public static final double kMaxArmRadiansPerSecond =
      Units.rotationsPerMinuteToRadiansPerSecond(kMaxArmSpeedRpm);

    public static final double kMaxArmSpeed = 0.8;
    //public static final double kArmSlewRate = 2;
    public static final double kArmDeadband = 0.1;

    public static final double kArmTestOffsetRads = Units.degreesToRadians(15);
    public static final double kArmShootingAngleRads = Units.degreesToRadians(57.5); //Amp shooting agngle
    public static final double kArmFarShootingAngleRads = Units.degreesToRadians(47.5); //Connect to chain
    public static final double kArmPickupAngleRads = Units.degreesToRadians(-39); //37.5 Normal Pickup & Auto Shoot
    //public static final double kArmStraightUpAngleRads = Units.degreesToRadians(90 );
    public static final double kArmDownRads = Units.degreesToRadians(-41); //-40 Jiggle & Trap 
    public static final double kArmShootingStepsRads = (kArmShootingAngleRads - kArmFarShootingAngleRads) / 3; //20
    public static final double kArmPickupStepsRads = Units.degreesToRadians(1); //20
    public static final double kArmFeederAngle = Units.degreesToRadians(-20); //20

  }
    
  public static final class HendersonConstants {
    public static final int kLeftLauncherMotorCanId = 1;
    public static final int kRightLauncherMotorCanId = 2;
    public static final int kLeftFeederMotorCanId = 3;
    public static final int kRightFeederMotorCanId = 4;

    public static final double kFeederGearRatio = 47/11;
    public static final double kIntakeLauncherSpeed = 0.1;
    public static final double kIntakeFeederSpeed = 0.3;
    public static double kLauncherIdleRpm = 1000;
    public static double kMaxLauncherRpm = 4000;

    public static double kLauncherP = 0.00025;
    public static double kLauncherI = 0.0;
    public static double kLauncherD = 0.00001;
    public static double kLauncherFF = 0.0001505;
    public static double kLauncherOutputMin = -1;
    public static double kLauncherOutputMax = 1;

    public static double kFeederBackSpeed = 1; // Feeder --> EXIT
    public static double kFeederFrontSpeed = -0.55; // Feeder --> Launcher
    public static double kLauncherFrontSpeed = 0.75; // Launcher --> EXIT
    public static double kLauncherBackSpeed = -0.75; // Launcher --> Feeder
    
    public static double kLauncherIntakeNoteSpeed = -0.75; // Launcher --> Feeder
    public static double kLauncherShootNoteBackSpeed = -0.75; // Launcher --> Feeder
    public static double kLauncherCenteringSpeed = 0.1; // Launcher --> EXIT




  }

  public static final class TagVisionConstants {
        public static final String kCameraName = "Tag Camera OV9281";
        public static final Transform3d kCameraOffset = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-5), // 1.0 in
                Units.inchesToMeters(-12), // -12 in
                Units.inchesToMeters(8.50)), //8.5 in
            new Rotation3d(
                0.0,
                Rotation2d.fromDegrees(20.0).getRadians(), //22
                Rotation2d.fromDegrees(175).getRadians()
            ));
        public static final double kMaxDistanceMeters = 3;
        public static final Pose2d kBlueSpeakerSubwoofer = 
          new Pose2d(1.45,5.55,new Rotation2d(Units.degreesToRadians(0)));
        public static final Pose2d kRedSpeakerSubwoofer = 
          new Pose2d(15.15,5.55,new Rotation2d(Units.degreesToRadians(180)));
    }

  public static final class NoteVisionConstants {
        public static final String kCameraName = "Note Camera OV9782";
        public static final Transform3d kCameraOffset = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(1.0),
                Units.inchesToMeters(-11.5),
                Units.inchesToMeters(24.50)),
            new Rotation3d(
                0.0,
                Rotation2d.fromDegrees(25.0).getRadians(),
                Rotation2d.fromDegrees(10).getRadians()
            ));
    }
}
