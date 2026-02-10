package frc.robot.commands.Coral;

import javax.naming.InsufficientResourcesException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.CoralSubsystem;

public class autoIntakeAngle extends Command {
  private final CoralSubsystem coral;
  private double intakeAngle = 125;
  private double speed;
  private boolean isFinished;
  private double angleMargin = 2;

  public autoIntakeAngle(CoralSubsystem coral, double speed, double intakeAngle) {
    this.coral = coral;
    this.speed = speed;
    this.intakeAngle = intakeAngle;
    addRequirements(coral);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("angle: " + coral.getAngle());
    System.out.println("wanted: " + intakeAngle);
    if(Math.abs(intakeAngle - coral.getAngle()) > angleMargin){
    coral.SetIntakeMotorSpeed(speed * Math.signum(intakeAngle - coral.getAngle()));
    System.out.println("Output: " + speed * Math.signum(intakeAngle - coral.getAngle()));

  }
    else{
      isFinished = true;
      
    }
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
