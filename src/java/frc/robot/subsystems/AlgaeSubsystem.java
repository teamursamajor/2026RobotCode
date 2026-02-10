package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeSubsystem extends Command {
    Spark turnMotor = new Spark(2);
    Spark motor = new Spark(1);

    double driveSpeed;
    double turnSpeed;
    
    public AlgaeSubsystem() {
        driveSpeed = -0.5;
        turnSpeed = 0.75;
    }

    public void startMotor() {
        
        motor.set(driveSpeed);
    }

    public void stopMotor() {
        motor.set(0);
        turnMotor.set(0);
    }

    public void increaseAngle() {
        
        turnMotor.set(turnSpeed);
    }

    public void decreaseAngle() {
        turnMotor.set(-turnSpeed);
    }
}
