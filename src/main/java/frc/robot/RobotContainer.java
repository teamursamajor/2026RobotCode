// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.JoyStickTestCommand;

//import frc.robot.commands.AprilTag.AprilTagTempTest;
//import frc.robot.commands.AprilTag.AprilTagTestCommand;
//import frc.robot.commands.AprilTag.AprilTagTestCommand2;

//import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
 * subsystems, commands, and trigger mappings) should be declared here.
 */
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller(s)
  XboxController m_driverController = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.

    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(Constants.DriveJoyStick.getY(), kDriveYDeadband),
    //             -MathUtil.applyDeadband(Constants.DriveJoyStick.getX(), kDriveXDeadband),
    //             -MathUtil.applyDeadband(Constants.DriveJoyStick.getZ() * 0.75, kRotDeadband),
    //             true),
    //         m_robotDrive));  



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
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

       


    //JoystickButton button1 = new JoystickButton(Constants.DriveJoyStick, 1);
    //JoystickButton button2 = new JoystickButton(Constants.DriveJoyStick, 2);
    //JoystickButton button3 = new JoystickButton(Constants.DriveJoyStick, 3);
   // JoystickButton button4 = new JoystickButton(Constants.DriveJoyStick, 4);
    //JoystickButton button5 = new JoystickButton(Constants.DriveJoyStick, 5);
    //JoystickButton button6 = new JoystickButton(Constants.DriveJoyStick, 6);
    //JoystickButton button7 = new JoystickButton(Constants.DriveJoyStick, 7);
    //JoystickButton button8 = new JoystickButton(Constants.DriveJoyStick, 8);
    
    //button1.whileTrue(new AlgeaIncreaseAngle(m_Algae));
    
    //button3.whileTrue(new CoralChangeAngle(coralSubsystem,0.3));
    //button5.whileTrue(new CoralChangeAngle(coralSubsystem,-0.3));
    //button4.whileTrue(new ClimbDownCommand(m_climb));
    //button5.whileTrue(new AlgeaIncreaseAngle(m_Algae));
    //button4.whileTrue(new PullCoral(coralSubsystem));
    //button6.whileTrue(new PushCoral(coralSubsystem));

    //button7.whileTrue(new ElevatorUp(elevatorSubsystem));
    //button8.whileTrue(new ElevatorDown(elevatorSubsystem));
    // Black Controller
    // Constants.xboxController.leftTrigger().whileTrue(new PullCoral(coralSubsystem));
    // Constants.xboxController.rightTrigger().whileTrue(new PushCoral(coralSubsystem));
    // Constants.xboxController.leftBumper().whileTrue(new IntakeAngle(coralSubsystem));
    // Constants.xboxController.rightBumper().whileTrue(new OutakeAngle(coralSubsystem));
    // Constants.xboxController.povLeft().whileTrue(new ElevatorSetHeight(elevatorSubsystem, 1));
    // Constants.xboxController.povUp().whileTrue(new ElevatorSetHeight(elevatorSubsystem, 4));
    // Constants.xboxController.povRight().whileTrue(new ElevatorSetHeight(elevatorSubsystem, 3));
    // Constants.xboxController.povDown().whileTrue(new ElevatorSetHeight(elevatorSubsystem, 2));
    // Constants.xboxController.b().whileTrue(new autoIntakeAngle(coralSubsystem, 0.3, 90));
    // Constants.xboxController.x().whileTrue(new autoIntakeAngle(coralSubsystem, 0.3, 135));
    // Constants.xboxController.y().whileTrue(new ElevatorUp(elevatorSubsystem));
    //onstants.xboxController.a().whileTrue(new ElevatorDown(elevatorSubsystem));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
