package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class FireBallsAtSetDistanceAutonomous extends Command{
    ShooterSubsystem m_shooter = new ShooterSubsystem();
    public double desiredShootSpeed = 10;
    public double AmountAboveDesiredShootSpeedToFeed = 2;

    public static double ShootSpeedToWheelSpeedRatio = 1;


    public FireBallsAtSetDistanceAutonomous(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    
    }
    
    @Override
    public void initialize(){
        double desiredWheelSpinSpeed = desiredShootSpeed * ShootSpeedToWheelSpeedRatio;
        m_shooter.ShooterSetSpeed(desiredWheelSpinSpeed);
    }
    @Override
    public void execute(){
            if(m_shooter.GetAvrageShooterSpeed() > desiredShootSpeed + AmountAboveDesiredShootSpeedToFeed){
                m_shooter.ShooterFeed(0.5);
            }
            else{
                m_shooter.ShooterFeed(0);
            }
    }

    @Override
    public void end(boolean interrupted){
        
    }
}
