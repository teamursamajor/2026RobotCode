package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorSetHeight extends Command {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final ElevatorSubsystem elevator_subsystem;

  public int heightLevel; // which level out of 4
  public float desiredHeight; // inches
  public static float heightToTurnRatio;

  public ElevatorSetHeight(ElevatorSubsystem subsystem, int heightLevel) {
    this.heightLevel = heightLevel;
    elevator_subsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator_subsystem.ElevatorSetHeight(heightLevel);
    elevator_subsystem.checkEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator_subsystem.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}