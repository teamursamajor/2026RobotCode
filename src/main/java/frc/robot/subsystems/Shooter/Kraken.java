package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;


public class Kraken {


    public static void SetupKraken(TalonFX kraken, double P, double I, double D, double FF) {
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = P;
        slot0Configs.kI = I;
        slot0Configs.kD = D;
        slot0Configs.kV = FF;

        kraken.getConfigurator().apply(slot0Configs);

    }

}
