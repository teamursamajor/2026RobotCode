package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;;

public class FireBallsWithAprilTagLocation extends Command{
    ShooterSubsystem m_shooter = new ShooterSubsystem();
    public double desiredDistance;
    public double desiredHeight;
    public double desiredShootSpeed = 0;
    public double AmountAboveDesiredShootSpeedToFeed = 2;
    public double shooterAngle = 70; //degrees
    public static double ShootSpeedToWheelSpeedRatio = 1;


    public FireBallsWithAprilTagLocation(ShooterSubsystem shooterSubsystem, double desiredDistance, double desiredHeight) {
    addRequirements(shooterSubsystem);
    this.desiredDistance = desiredDistance;
    this.desiredHeight = desiredHeight;
    }
    public double ShootVelocityToHitPoint(double x, double y){
        double velocity = 1;
        double gravity = 9.8;
        velocity = Math.sqrt(((gravity*Math.pow(x,2)*Math.pow(Math.tan(gravity),2)) + gravity * Math.pow(x,2)) / (2 * (-y + (x * Math.tan(shooterAngle)))));
        return velocity;
    }
    @Override
    public void initialize(){
        desiredShootSpeed = ShootVelocityToHitPoint(desiredDistance, desiredHeight);
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
