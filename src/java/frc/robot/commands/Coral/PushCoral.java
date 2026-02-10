package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.CoralSubsystem;

public class PushCoral extends Command {

  private CoralSubsystem pushcoral;

  // Called when the command is initially scheduled.
  public PushCoral(CoralSubsystem neoSubsystem) {
    pushcoral = neoSubsystem;
    addRequirements(neoSubsystem);

  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pushcoral.push();
    // System.out.println(pushcoral.getAngle());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pushcoral.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}