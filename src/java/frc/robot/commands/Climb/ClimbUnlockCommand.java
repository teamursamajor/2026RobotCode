package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.ClimbSubsystem;

public class ClimbUnlockCommand extends Command {
  private ClimbSubsystem climbsubsystem;
  private boolean isFinished = false;

  public ClimbUnlockCommand(ClimbSubsystem climbSubsystem) {
    this.climbsubsystem = climbSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climbsubsystem.unlockServo();
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Climb Unlocked");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

}
