package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.CoralSubsystem;

public class OutakeAngle extends Command {
  private CoralSubsystem coral;
  private double outakeAngle = 90;
  private double angleMargin = 1;
  private double speed = 0.25;
  private boolean isFinished;

  public OutakeAngle(CoralSubsystem coral) {
    this.coral = coral;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("outake Angle: " + coral.getAngle());
    if(coral.getAngle() > outakeAngle + angleMargin){
      speed = 0.35;
    coral.SetIntakeMotorSpeed(speed);
    } else if (coral.getAngle() < outakeAngle - angleMargin){
      speed = 0.25;
    coral.SetIntakeMotorSpeed(-speed);
    } else {
    isFinished = true;
    }

    //coral.setAngleMotor(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
