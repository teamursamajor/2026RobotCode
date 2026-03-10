package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    SparkMax kIntakePitchTalonFXid = new SparkMax(Constants.kIntakePitchTalonFXid, null);
    SparkMax kIntakePullingTalonFXid = new SparkMax(Constants.kIntakePullingTalonFXid, null);

    public IntakeSubsystem(){
        

    }
}
