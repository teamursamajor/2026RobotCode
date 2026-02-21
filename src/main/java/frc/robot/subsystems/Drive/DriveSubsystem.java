package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.SwerveUtils;
import com.studica.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  AHRS ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);

  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(Constants.kWheelBase / 2, -Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, Constants.kTrackWidth / 2),
      new Translation2d(-Constants.kWheelBase / 2, -Constants.kTrackWidth / 2));

  // Create MAXSwerveModules
  public final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      Constants.kFrontLeftDrivingTalonId,
      Constants.kFrontLeftTurningId,
      (-Math.PI / 2));

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      Constants.kFrontRightDrivingTalonId,
      Constants.kFrontRightTurningId,
      0);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      Constants.kBackLeftDrivingTalonId,
      Constants.kBackLeftTurningId,
      Math.PI);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      Constants.kBackRightDrivingTalonId,
      Constants.kBackRightTurningId,
      (Math.PI / 2));

  // The gyro sensor

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;
  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(1.8); // percent per second (1 = 100%)
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(2.0); // percent per second (1 = 100%)
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      kDriveKinematics,
      Rotation2d.fromDegrees(ahrs.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */

  public DriveSubsystem() {
    // ahrs.setAngleAdjustment(180);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(ahrs.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(ahrs.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
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

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = xSpeed * .75;
    ySpeed = ySpeed * .75;
    rot = rot * .75;

    // System.out.println("Swerve Drive");
    if (Math.abs(xSpeed) < .1) {
      xSpeed = 0.0;
    }
    if (Math.abs(ySpeed) < .1) {
      ySpeed = 0.0;
    }
    if (Math.abs(rot) < .1) {
      rot = 0.0;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * Constants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * Constants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * Constants.kMaxAngularSpeed;

    var swerveModuleStates = kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-ahrs.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("gyroAngle", ahrs.getAngle());
    SmartDashboard.putNumber("gyro pitch", ahrs.getPitch());
    SmartDashboard.putNumber("gyro Yaw", ahrs.getYaw());
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
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, Constants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setLeftDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setLeftDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */

  public void zeroHeading() {
    ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */

  public double getHeading() {
    return Rotation2d.fromDegrees(ahrs.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */

  public double getTurnRate() {
    return ahrs.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

}