package frc.robot.subsystems.AprilTag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.*;

public class AprilTagAlign {

    double yaw;
    double pitch;
    double distanceX;
    double distanceY;
    double id;
    double y;

    public AprilTagAlign(double yaw, double pitch, double distanceX, double distanceY, double id) {
        this.yaw = yaw;
        this.pitch = pitch;
        this.distanceX = distanceX;
        this.distanceY = distanceY;
        this.id = id;
        this.y = y;
    }

    public double getYaw() {
        return yaw;
    }

    public double getPitch() {
        return pitch;
    }

    public double getDistanceX() {
        return distanceX;
    }

    public double getDistanceY() {
        return distanceY;
    }

    public double getId() {
        return id;
    }

    public double getY() {
        return y;
    }
}
