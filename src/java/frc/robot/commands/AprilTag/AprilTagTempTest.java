package frc.robot.commands.AprilTag;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTag.AprilTagAlign;
import frc.robot.subsystems.AprilTag.AprilTagSubsystem;
//import frc.robot.subsystems.Drive.DriveSubsystem;

public class AprilTagTempTest extends Command {



    AprilTagSubsystem aprilTag = new AprilTagSubsystem();
    AprilTagAlign target;
    //DriveSubsystem m_drive;

    public AprilTagTempTest(AprilTagSubsystem apriltag){
        this.aprilTag = apriltag;
        //this.m_drive = m_drive;
    }

    @Override
    public void execute(){
        target = aprilTag.targetValues();
        SmartDashboard.putNumber("Y distance", target.getDistanceX());
        SmartDashboard.putNumber("Yaw angle", target.getYaw());
        SmartDashboard.putNumber("X distance", target.getDistanceX());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
