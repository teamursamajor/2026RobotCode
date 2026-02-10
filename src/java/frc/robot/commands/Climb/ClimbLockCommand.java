package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.ClimbSubsystem;

public class ClimbLockCommand extends Command {
  private ClimbSubsystem climbSubsystem;
  private boolean isFinished = false;

  public ClimbLockCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbSubsystem.lockServo();
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Climb Locked");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
