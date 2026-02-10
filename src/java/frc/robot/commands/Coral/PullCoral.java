package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.CoralSubsystem;

public class PullCoral extends Command {

  private final CoralSubsystem pullcoral;

  public PullCoral(CoralSubsystem coralSubsystem) {
    pullcoral = coralSubsystem;
    addRequirements(coralSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pullcoral.pull();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pullcoral.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
