package frc.robot.subsystems.AprilTag;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.*;

public class AprilTagSubsystem extends SubsystemBase {
    // double cameraHeight = Units.inchesToMeters(57.75);
    double testHeight = Units.inchesToMeters(23.5);
    double testcamHeight = Units.inchesToMeters(19);
    double cameraPitchRadians = Units.degreesToRadians(0); // Angle between horizontal and the camera.

    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera"); // Change to camera name

    public AprilTagSubsystem() {

    }

    public AprilTagAlign targetValues() {
        // Vision-alignment mode
        // Query the latest result from PhotonVision
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        // System.out.println(camera.getName() + " " + camera.getPipelineIndex() + " " +
        // camera.isConnected() + " ");

        double yaw = Double.MAX_VALUE;
        double pitch = Double.MAX_VALUE;
        double id = Double.MAX_VALUE;
        double distanceX = Double.MAX_VALUE;
        double distanceY = Double.MAX_VALUE;

        // System.out.println("results: " + results.isEmpty());
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                Transform3d bestCameraToTarget = target.getBestCameraToTarget();
                // System.out.println("X:" + Units.metersToInches(bestCameraToTarget.getX()));

                yaw = Units.radiansToDegrees(bestCameraToTarget.getRotation().getMeasureZ().magnitude());
                pitch = target.getPitch();
                id = target.getFiducialId();
                distanceX = bestCameraToTarget.getX();
                distanceY = bestCameraToTarget.getY();
                SmartDashboard.putNumber("DistanceX", distanceX);
                SmartDashboard.putNumber("DistanceY", distanceY);
                SmartDashboard.putNumber("Yaw", yaw);

                System.out.println(
                        "get yaw" + Units.radiansToDegrees(bestCameraToTarget.getRotation().getMeasureZ().magnitude()));

                System.out.println("get angle " + Units.radiansToDegrees(bestCameraToTarget.getRotation().getAngle()));
            }

        }
        return new AprilTagAlign(yaw, pitch, distanceX, distanceY, id);
    }
}