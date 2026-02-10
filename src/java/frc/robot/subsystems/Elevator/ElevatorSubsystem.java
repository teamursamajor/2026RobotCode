package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
//import frc.robot.Constants.DriveConstants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

public class ElevatorSubsystem extends SubsystemBase {
    SparkMax motor1 = new SparkMax(6, MotorType.kBrushless);
    SparkMaxConfig Config = new SparkMaxConfig();
    public RelativeEncoder m_encoder1;

    public float motorSpeed = 0.5f;
    public float turnTolerence = 0.2f;
    public int heightLevel;
    public double desiredHeight; // inches
    public static float heightToTurnRatio = 3.927f / 48; // inches to rotations, gear ratio
    public double desiredTurns;
    public double elevatorBaseHeight; // inches, height that elevator is at lowest
    double currentTurns;
    private double margin = 2.5;
    private double speed = 0.5;
    PIDController pid = new PIDController(0.1, 0, 0);

    public ElevatorSubsystem() {
        m_encoder1 = motor1.getEncoder();
        currentTurns = m_encoder1.getPosition();
        SmartDashboard.putNumber("encoder: ", m_encoder1.getPosition());
        

    }

    public void ElevatorSetHeight(int heightLevel) {
        

        // Config.closedLoop
        //         .pidf(0.1, 0, 0, 0)
        // .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // .positionWrappingEnabled(true)
        // .outputRange(-1, 1);
        // motor1.configure(Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        if (heightLevel < 1) {
            heightLevel = 1;
        }
        if (heightLevel > 4) {
            heightLevel = 4;
        }

        if (heightLevel == 1) {
            desiredHeight = 0; // 1 foot 6 inches

        } else if (heightLevel == 2) {
            desiredHeight = 100; // 2 foot 7/8 inches

        } else if (heightLevel == 3) {
            desiredHeight = 230; // 3 foot 11+5/8 inches

        } else if (heightLevel == 4) {
            desiredHeight = 544; // 6 foot

        }

        //System.out.println("desired height: " + desiredHeight);
      
        if (Math.abs(desiredHeight - -m_encoder1.getPosition()) > margin) {
            //motor1.set(pid.calculate(m_encoder1.getPosition(), 100));
            motor1.set(-0.5 * Math.signum(desiredHeight - -m_encoder1.getPosition()));
            //System.out.println("motor speed: " + pid.calculate(m_encoder1.getPosition(), 100));
        } else {
            motor1.set(0);
        }

    }
    
    // motor1.set(pid.calculate(m_encoder1.getPosition(), 100));
    public void elevatorUp(double upSpeed) {
        motor1.set(-upSpeed);
    }

    public void elevatorDown(double downSpeed) {
        motor1.set(downSpeed);
    }

    public void elevatorReset() {
        m_encoder1.setPosition(0);
    }

    public void checkEncoder() {
        System.out.println("encoder: " + -m_encoder1.getPosition());
    }

    public void stopElevator() {
        motor1.set(0);
    }

}
