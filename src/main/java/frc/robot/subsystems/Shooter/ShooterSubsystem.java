package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    public TalonFX m_feedTalonFX;
    public TalonFX m_FrontShootTalonFX;
    public TalonFX m_BackShootTalonFX;

    
    
    public ShooterSubsystem(){
        m_feedTalonFX = new TalonFX(Constants.kfeedTalonFXid);
        m_FrontShootTalonFX = new TalonFX(Constants.kBackShootTalonFXid);
        m_BackShootTalonFX = new TalonFX(Constants.kFrontShootTalonFXid);
        Kraken.SetupKraken(m_feedTalonFX, 1,0,0,0);
        Kraken.SetupKraken(m_FrontShootTalonFX,1,0,0,0);
        Kraken.SetupKraken(m_BackShootTalonFX,1,0,0,0);
        
        
    
    }
    public void ShooterFeed(double speed){
        m_feedTalonFX.set(speed);
        m_feedTalonFX.set(speed);
        
    }
    public void ShooterSetSpeed(double speed){
        VelocityVoltage FrontShooterVelocityVoltage = new VelocityVoltage(30);
        m_feedTalonFX.setControl(FrontShooterVelocityVoltage);
        VelocityVoltage BackShooterVelocityVoltage = new VelocityVoltage(-30);
        m_feedTalonFX.setControl(BackShooterVelocityVoltage);
        
        
    }
    public double GetAvrageShooterSpeed(){
        double FrontShootTalonVel = Math.abs(m_FrontShootTalonFX.getVelocity().getValueAsDouble());
        double BackShootTalonVel = Math.abs(m_BackShootTalonFX.getVelocity().getValueAsDouble());
        return (FrontShootTalonVel+BackShootTalonVel)/2;
        }

    public void VelocityTest(){
        
    }
    public void SpinAllMotors(){
        m_FrontShootTalonFX.set(1);
        m_BackShootTalonFX.set(1);
        m_feedTalonFX.set(1);
    }
    public void StopAllMotors(){
        m_FrontShootTalonFX.set(0);
        m_BackShootTalonFX.set(0);
        m_feedTalonFX.set(0);
    }
}
