// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Intake.IntakeDrop;
import frc.robot.commands.Intake.IntakeFeed;
import frc.robot.commands.Auto.MoveDirections;
import frc.robot.commands.Shooter.FireBallsAtSetDistance;
import frc.robot.commands.Shooter.ShooterTestMotors;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

import com.studica.frc.jni.AHRSJNI;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappgs) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
  public static final double kTrackWidth = Units.inchesToMeters(27.5);
  // Distance between centers of right and left wheels on robot
  public static final double kWheelBase = Units.inchesToMeters(28.0);

  public final double kMaxSpeedMetersPerSecond = 4.8;
  public final double kMaxAccelerationMetersPerSecondSquared = 3;
  public final double kPXController = 1;
  public final double kPYController = 1;
  public final double kPThetaController = 1;
  public final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
  public final double kDriveYDeadband = 0.03;
  public final double kDriveXDeadband = 0.1;
  public final double kRotDeadband = 0.4;

  // The robot's subsystems and commands are defined here...
  public DriveSubsystem m_robotDrive = new DriveSubsystem();
  public ShooterSubsystem m_shooter = new ShooterSubsystem();
  public IntakeSubsystem m_Intake = new IntakeSubsystem();
  // The driver's controller(s)
  // XboxController m_driverController = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // // // The left stick controls translation of the robot.
        // // // Turning is controlled by the X axis of the right stick.

        new RunCommand(
              

            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Constants.DriveJoyStick.getY(), kDriveYDeadband),
                -MathUtil.applyDeadband(Constants.DriveJoyStick.getX(), kDriveXDeadband),
                -MathUtil.applyDeadband(Constants.DriveJoyStick.getZ(), kRotDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Contro+ller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    JoystickButton button1 = new JoystickButton(Constants.DriveJoyStick, 1);
    JoystickButton button2 = new JoystickButton(Constants.DriveJoyStick, 2);
    JoystickButton button3 = new JoystickButton(Constants.DriveJoyStick, 3);
    JoystickButton button4 = new JoystickButton(Constants.DriveJoyStick, 4);
    JoystickButton button5 = new JoystickButton(Constants.DriveJoyStick, 5);
    JoystickButton button6 = new JoystickButton(Constants.DriveJoyStick, 6);
    JoystickButton button7 = new JoystickButton(Constants.DriveJoyStick, 7);
    JoystickButton button8 = new JoystickButton(Constants.DriveJoyStick, 8);

    button1.whileTrue(new FireBallsAtSetDistance(m_shooter));

    //button2.whileTrue(new IntakeFeed(m_Intake));
    button5.whileTrue(new IntakeDrop(m_Intake));
    
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new MoveDirections(m_robotDrive, -1,0,0).withTimeout(0.4).andThen(new FireBallsAtSetDistance(m_shooter).withTimeout(2));

  }

}
