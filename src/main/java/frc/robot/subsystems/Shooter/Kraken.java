package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Kraken {
    public static final IdleMode krakenIdleMode = IdleMode.kBrake;
    public static final double krakenEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double krakenEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static void SetupKraken(TalonFX kraken, double P, double I, double D, double FF) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = P;
        slot0Configs.kI = I;
        slot0Configs.kD = D;
        slot0Configs.kV = FF;

        kraken.getConfigurator().apply(slot0Configs);

    }

}
