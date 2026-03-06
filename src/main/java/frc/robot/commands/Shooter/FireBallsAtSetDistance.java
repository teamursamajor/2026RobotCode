package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;;

public class FireBallsAtSetDistance extends Command{
    ShooterSubsystem m_shooter = new ShooterSubsystem();
    public double desiredShootSpeed = 100;
    public double AmountBelowDesiredShootSpeedToFeed = 10;

    public static double ShootSpeedToWheelSpeedRatio = 1;

    public FireBallsAtSetDistance(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
    
    }
    
    @Override
    public void initialize(){
        double desiredWheelSpinSpeed = desiredShootSpeed * ShootSpeedToWheelSpeedRatio;
        m_shooter.ShooterSetSpeed(desiredWheelSpinSpeed);
    }
    @Override
    public void execute(){
        System.out.println(m_shooter.GetAvrageShooterSpeed());
            if(m_shooter.GetAvrageShooterSpeed() > desiredShootSpeed - AmountBelowDesiredShootSpeedToFeed){
                m_shooter.ShooterFeed(1);
            }
            else{
                m_shooter.ShooterFeed(0);
            }
    }

    @Override
    public void end(boolean interrupted){
        m_shooter.StopAllMotors();
    }


}
