package frc.robot.commands.Coral;

import javax.naming.InsufficientResourcesException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.CoralSubsystem;

public class IntakeAngle extends Command {
  private final CoralSubsystem coral;
  private double intakeAngle = 125;
  private double speed = 0.25;
  private boolean isFinished;
  private double angleMargin = 1;

  public IntakeAngle(CoralSubsystem coral) {
    this.coral = coral;
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
    
    if (coral.getAngle() > intakeAngle + angleMargin) {
      speed = 0.1;
    coral.SetIntakeMotorSpeed(speed);
    } else if (coral.getAngle() < intakeAngle - angleMargin) {
      speed = 0.1;
    coral.SetIntakeMotorSpeed(-speed);
    } else {
    isFinished = true;
    }
    System.out.println(coral.getAngle() + " degrees twin");
    System.out.println(intakeAngle + " intake angle twin");
    //coral.setAngleMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coral.stopMotor();
    //System.out.println("intake angle done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

}
