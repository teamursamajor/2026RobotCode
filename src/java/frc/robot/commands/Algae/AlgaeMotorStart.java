package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSubsystem;

public class AlgaeMotorStart extends Command {
    private AlgaeSubsystem algae;

    public AlgaeMotorStart(AlgaeSubsystem algaeSubsystem){
        this.algae = algaeSubsystem;
    }

    
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algae.startMotor();
    System.out.println("extra button");
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
