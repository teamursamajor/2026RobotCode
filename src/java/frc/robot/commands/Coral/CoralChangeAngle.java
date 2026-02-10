package frc.robot.commands.Coral;

//import javax.naming.InsufficientResourcesException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral.CoralSubsystem;

public class CoralChangeAngle extends Command{
    private double speed;
    private final CoralSubsystem coral;
    public CoralChangeAngle(CoralSubsystem coral, double speed) {
        this.coral = coral;
        this.speed = speed;
        addRequirements(coral);
      }

      public void execute() {
        coral.SetIntakeMotorSpeed(speed);
      
        
    }
      public void end(boolean interrupted) {
        coral.stopMotor();
      }
}

