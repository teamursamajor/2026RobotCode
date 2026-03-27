package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
   public IntakeSubsystem() {

   }

   Spark redLineToBringIntakeUpDown = new Spark(Constants.kIntakeArm);
   Spark redLineToConsume = new Spark(Constants.kIntakeConsume);

   public void IntakeSetSpeed(double speed) {
      redLineToConsume.set(speed);
   }

   public void IntakeDropArm() {
      redLineToBringIntakeUpDown.set(1);
   }

   public void IntakeStopArm() {
      redLineToBringIntakeUpDown.set(0);
   }

}
