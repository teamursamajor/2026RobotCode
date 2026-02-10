package frc.robot.commands.Auto;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
//import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.math.util.Units;

public class LeaveCommand extends Command {
  private DriveSubsystem m_drive;
  private boolean isFinished = false;
  private long time;

  public LeaveCommand(DriveSubsystem driveSubsystem) {
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
    long currentTime = RobotController.getFPGATime() - time;
    // m_drive.drive(0, 0, 0.25,false);
    if (true) {
      m_drive.drive(-0.25, 0, 0, false);
    }
    if (currentTime > (2 * 1000000)) {
      isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;

  }

}
