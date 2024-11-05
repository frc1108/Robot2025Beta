// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.utils.SwerveUtils;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class DriveSubsystem extends SubsystemBase implements Logged {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontLeftDrivingkP,
      DriveConstants.kFrontLeftDrivingkS,
      DriveConstants.kFrontLeftDrivingkV,
      DriveConstants.kFrontLeftDrivingkA);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kFrontRightDrivingkP,
      DriveConstants.kFrontRightDrivingkS,
      DriveConstants.kFrontRightDrivingkV,
      DriveConstants.kFrontRightDrivingkA);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      DriveConstants.kRearLeftDrivingkP,
      DriveConstants.kRearLeftDrivingkS,
      DriveConstants.kRearLeftDrivingkV,
      DriveConstants.kRearLeftDrivingkA);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      DriveConstants.kRearRightDrivingkP,
      DriveConstants.kRearRightDrivingkS,
      DriveConstants.kRearRightDrivingkV,
      DriveConstants.kRearRightDrivingkA);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(0);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private final PhotonCamera noteCam = new PhotonCamera("Note Camera OV9782");
  private final PhotonPipelineResult noteTarget = new PhotonPipelineResult();
  private final Constraints pidConstraints = new Constraints(3, 3);
  private final ProfiledPIDController pidX = new ProfiledPIDController(5.0, 0, 0, pidConstraints);
  private final ProfiledPIDController pidY = new ProfiledPIDController(5.0, 0, 0, pidConstraints);
  private final PIDController pidRot = new PIDController(5.0, 0, 0);

  // Odometry class for tracking robot pose
  private SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      //(DriverStation.getAlliance().get() == Alliance.Blue)?TagVisionConstants.kBlueSpeakerSubwoofer:TagVisionConstants.kRedSpeakerSubwoofer
      new Pose2d());

  @Log.NT(key = "Field") private final Field2d m_field = new Field2d();
  @Log.NT(key = "Vision Enabled") private boolean isVisionAdded = true;
  @Log.NT(key = "Vision Target Added") private boolean m_visionAdded = false;

  //private final PhotonCamera noteCamera = new PhotonCamera(NoteVisionConstants.kCameraName);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      new HolonomicPathFollowerConfig(
        AutoConstants.kTranslationPid,
        AutoConstants.kRotationPid,
        DriveConstants.kMaxSpeedMetersPerSecond, // max speed in m/s
        Math.sqrt(Math.pow(DriveConstants.kTrackWidth, 2)+Math.pow(DriveConstants.kWheelBase,2))/2, // Radius in meters of 28.5 x 18.5 inch robot using a^2 +b^2 = c^2
        new ReplanningConfig()
      ),
      ()->{// Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, // should flip path boolean supplier
      this
    );

    //m_odometry.setVisionMeasurementStdDevs(null);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_field.setRobotPose(m_odometry.getEstimatedPosition());

    // Log current speed of drive motors
    // this.log("LF Speed",Math.abs(m_frontLeft.getState().speedMetersPerSecond));
    // this.log("RL Speed",Math.abs(m_rearLeft.getState().speedMetersPerSecond));
    // this.log("FR Speed",Math.abs(m_frontRight.getState().speedMetersPerSecond));
    // this.log("RR Speed",Math.abs(m_rearRight.getState().speedMetersPerSecond));


  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  @Log.NT(key = "Robot Pose")
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  @Log.NT(key = "Speaker pose")
  public Pose2d getSpeakerPose() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return new Pose2d(0.0, 
                        Units.inchesToMeters(219.277), new Rotation2d());
    } else {
      return new Pose2d(Units.inchesToMeters(651.223),
                        Units.inchesToMeters(219.277), 
                        new Rotation2d(Units.degreesToRadians(180)));
    }  
  }

  @Log.NT(key = "Speaker distance meters")
  public double getDistanceToSpeakerMeters() {
    return PhotonUtils.getDistanceToPose(getPose(), 
    getSpeakerPose());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void visionPose(Pose2d pose,double timestamp){
    if (isVisionAdded) {
        m_odometry.addVisionMeasurement(pose, timestamp);
        m_visionAdded = true;
      // }
    }
  }

  public void stopVisionPose(){
    isVisionAdded = false;
  }

  public void startVisionPose(){
    isVisionAdded = true;
  }

  @Log.NT(key = "Vision Enabled")
  public boolean getVisionAdded(){
    return isVisionAdded;
  }

  public void setVisionStdDevs(double xMeters, double yMeters, double thetaRads) {
    m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xMeters,yMeters,thetaRads));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveChassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getPose().getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);
    //this.log("DesiredChassisSpeed",swerveChassisSpeeds);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(swerveChassisSpeeds);
    setStates(swerveModuleStates);
    //this.log("DesiredModuleStates",swerveModuleStates);
    // this.log("FL Setpoint",swerveModuleStates[0].speedMetersPerSecond);
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // this.log("FR Setpoint",swerveModuleStates[1].speedMetersPerSecond);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // this.log("RL Setpoint",swerveModuleStates[2].speedMetersPerSecond);
    // m_rearLeft.setDesiredState(swerveModuleStates[2]);
    // this.log("RR Setpoint",swerveModuleStates[3].speedMetersPerSecond);
    // m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] targetStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DriveConstants.kMaxSpeedMetersPerSecond);
    //this.log("RealDesiredChassisSpeed",DriveConstants.kDriveKinematics.toChassisSpeeds(targetStates));
    //this.log("DesiredModuleStates",targetStates);
    m_frontLeft.setDesiredState(targetStates[0]);
    m_frontRight.setDesiredState(targetStates[1]);
    m_rearLeft.setDesiredState(targetStates[2]);
    m_rearRight.setDesiredState(targetStates[3]);
  }


  // public void driveRobotRelative(ChassisSpeeds speeds){
  //   this.drive(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false,false);
  // }
  @Log.NT(key = "Note Distance")
  public double distanceToNote(){
    if (noteTarget.hasTargets()) {
      var result = noteTarget.getBestTarget();
      return PhotonUtils.calculateDistanceToTargetMeters(Units.inchesToMeters(11.5),
                                                0, //Note on ground
                                                Units.degreesToRadians(-30),
                                                result.getPitch());
    } else {
      return -1;
    }
  }
  
  @Log.NT(key = "Chassis Speed")
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(),
                                                           m_frontRight.getState(),
                                                           m_rearLeft.getState(),
                                                           m_rearRight.getState());
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }
  /**
   * Sets the wheels forward.
   */
  public Command setAllWheelsForward() {
    return Commands.run(()->{
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    },this);
  }
  /**
   * Sets the wheels right.
   */
  public Command setAllWheelsRight() {
    return Commands.run(()->{
      m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-90)));
      m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-90)));
      m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-90)));
      m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-90)));
    },this);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  @Log.NT(key = "Heading, deg")
  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }


  public Command driveToNote() {
    return Commands.none();
  }
}
