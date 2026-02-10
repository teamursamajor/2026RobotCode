package frc.robot.subsystems.Climb;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ClimbSubsystem extends SubsystemBase {
    public Timer timer = new Timer();

    private Spark motor = new Spark(4);
    private Servo servo = new Servo(0);

    public ClimbSubsystem() {
        servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000); // 2000 1800 1500 1200 1000

    }

    public void lockServo() {
        // servo.setSpeed(1.0);
        servo.set(0.2);
        // System.out.println("lock");
    }

    public void unlockServo() {
        // servo.setSpeed(-1.0);
        servo.set(0.4);
        // System.out.println("unlock");
    }

    public void climbUp() {
        motor.set(-.25);
    }

    public void climbDown() {
        motor.set(.75);
        System.out.println(motor.getVoltage());
    }

    public void stopClimb() {
        // System.out.println("climb stopped");
        motor.set(0);

    }

    public double getPosition() {
        // SmartDashboard.putNumber("Servo Position", servo.get());
        return servo.get();
    }
}
