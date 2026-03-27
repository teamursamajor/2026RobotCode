package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeFeed extends Command {
    IntakeSubsystem m_intake;

    public IntakeFeed(IntakeSubsystem intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_intake.IntakeSetSpeed(0.5);


    }

    @Override
    public void end(boolean interrupted) {
        m_intake.IntakeSetSpeed(0);
    }
}