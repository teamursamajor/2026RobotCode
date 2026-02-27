package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;;

public class ShooterTestMotors extends Command{
    ShooterSubsystem m_shooter = new ShooterSubsystem();

    public ShooterTestMotors(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);

    
    }
    
    @Override
    public void initialize(){
        System.out.println("initialized");
    }
    @Override
    public void execute(){
        m_shooter.SpinAllMotors();

    }

    @Override
    public void end(boolean interrupted){
               m_shooter.StopAllMotors();
    }

 
}