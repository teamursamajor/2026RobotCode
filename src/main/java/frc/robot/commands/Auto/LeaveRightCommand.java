package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class LeaveRightCommand extends Command {

  private DriveSubsystem m_drive;
  private boolean isFinished = false;
  private long time;

  public LeaveRightCommand(DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem);
    m_drive = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = RobotController.getFPGATime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Place Holder Values

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;

  }

}
