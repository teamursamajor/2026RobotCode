package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class IntakeSubsystem {
    public IntakeSubsystem(){
         
        

    }
 Spark redLineToBringIntakeUpDown = new Spark(Constants.kIntakeArm);
 Spark redLineToConsume = new Spark(Constants.kIntakeConsume);
 public void IntakeSetSpeed(double speed) {
    redLineToConsume.set(speed);
 }
 public void IntakeDropArm() {
    redLineToBringIntakeUpDown.set(.1);
 }
}
