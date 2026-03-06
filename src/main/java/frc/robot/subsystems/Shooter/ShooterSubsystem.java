package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    public TalonFX m_feedTalonFX;
    public TalonFX m_FrontShootTalonFX;
    public TalonFX m_BackShootTalonFX;

    VelocityVoltage FrontShooterVelocityVoltage;
    VelocityVoltage BackShooterVelocityVoltage;
    VelocityVoltage FeedShooterTalonFX;

    
    public ShooterSubsystem(){
        m_feedTalonFX = new TalonFX(Constants.kfeedTalonFXid);
        m_FrontShootTalonFX = new TalonFX(Constants.kBackShootTalonFXid);
        m_BackShootTalonFX = new TalonFX(Constants.kFrontShootTalonFXid);
        Kraken.SetupKraken(m_feedTalonFX, 1,0,0,1);
        Kraken.SetupKraken(m_FrontShootTalonFX,1,0,0,1);
        Kraken.SetupKraken(m_BackShootTalonFX,1,0,0,1);
        
        
    
    }
    public void ShooterFeed(double speed){
        m_feedTalonFX.set(-speed);
        
    }
    public void ShooterSetSpeed(double speed){
        FrontShooterVelocityVoltage = new VelocityVoltage(-speed);
        m_FrontShootTalonFX.setControl(FrontShooterVelocityVoltage);
        BackShooterVelocityVoltage = new VelocityVoltage(speed);
        m_BackShootTalonFX.setControl(BackShooterVelocityVoltage);
        
        
    }
    public double GetAvrageShooterSpeed(){
        double FrontShootTalonVel = Math.abs(m_FrontShootTalonFX.getVelocity().getValueAsDouble());
        double BackShootTalonVel = Math.abs(m_BackShootTalonFX.getVelocity().getValueAsDouble());
        return (FrontShootTalonVel+BackShootTalonVel)/2;
        }

    public void VelocityTest(){
        
    }
    public void SpinAllMotors(){
        m_feedTalonFX.setControl(FrontShooterVelocityVoltage);
        m_feedTalonFX.setControl(BackShooterVelocityVoltage);
        
    }
    public void StopAllMotors(){
        FrontShooterVelocityVoltage = new VelocityVoltage(0);
        BackShooterVelocityVoltage = new VelocityVoltage(0);
        FeedShooterTalonFX = new VelocityVoltage(0);

        m_FrontShootTalonFX.setControl(FrontShooterVelocityVoltage);
        m_BackShootTalonFX.setControl(FrontShooterVelocityVoltage);
        m_feedTalonFX.setControl(FrontShooterVelocityVoltage);

        m_FrontShootTalonFX.stopMotor();
        m_BackShootTalonFX.stopMotor();
        m_feedTalonFX.stopMotor();
        
    }
}
