package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgeaIncreaseAngle extends Command {
    private AlgaeSubsystem algae;

    public AlgeaIncreaseAngle(AlgaeSubsystem algaeSubsystem){
        this.algae = algaeSubsystem;
    }

    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.increaseAngle();
    //algae.startMotor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algae.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
