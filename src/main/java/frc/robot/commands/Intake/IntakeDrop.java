package frc.robot.commands.Intake;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDrop extends Command {
    IntakeSubsystem m_intake = new IntakeSubsystem();
    
    
    public IntakeDrop (IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
    }
     
    
    @Override
    public void initialize(){
        System.out.println("initialized");
    }
    @Override
    public void execute(){
        m_intake.IntakeDropArm();

    }

    @Override
    public void end(boolean interrupted){
               m_intake.IntakeStopArm();
    }
}
