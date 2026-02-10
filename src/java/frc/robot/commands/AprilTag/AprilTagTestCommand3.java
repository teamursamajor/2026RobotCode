package frc.robot.commands.AprilTag;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.AprilTag.AprilTagAlign;
import frc.robot.subsystems.AprilTag.AprilTagSubsystem;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class AprilTagTestCommand3 extends Command {

    AprilTagSubsystem aprilTag = new AprilTagSubsystem();

    DriveSubsystem m_drive;

    public AprilTagTestCommand3( AprilTagSubsystem apriltag, DriveSubsystem m_drive){
        this.aprilTag = apriltag;
        this.m_drive = m_drive;
    }

    private double yDesiredDistance = -0.4;
    private double xDesiredDistance = 1.5;
    private double yawDesiredAngle = 180;

    private double yDesDistMargin = 0.2;
    private double xDesDistMargin = 0.4;
    private double yawDesDistMargin = 7;

    private double motorSpeedMult = 0.45;
    private double turnSpeedMult = 0.25;

    private boolean isFinished;
    private long time;
    AprilTagAlign target;
    @Override
    public void initialize(){
        isFinished = false;
        time = RobotController.getFPGATime();
    }
    
    @Override
    public void execute() {
        long currentTime = RobotController.getFPGATime() - time;

        if(currentTime > 10 * (1000000)){
            System.out.println("Timed out (took to long)");
            isFinished = true;
            
        }
        System.out.println("Execute");
        target = aprilTag.targetValues();
        //System.out.println("id: " + target.getId());
        if (target.getId() != Double.MAX_VALUE) {
            SmartDashboard.putNumber("Y distance", target.getDistanceY());
            SmartDashboard.putNumber("Yaw angle", target.getYaw());
            SmartDashboard.putNumber("X distance", target.getDistanceX());
            

            if (inYMargin() == false) {
                alignY();
                //System.out.println("1");
            } else if (inYawMargin() == false) {
                //alignYaw();
                alignYaw();
                //System.out.println("2");
            } else if (inXMargin() == false) {
                alignX();
                //System.out.println("3");
            } else if (inXMargin() == true) {
                
                //m_drive.drive(0, 0, 0.0, false);
                
                isFinished = true;
            }

            //System.out.println("Yaw" + target.getYaw());
            //System.out.println("Y" + target.getDistanceY());
        }
    }

    public void alignY() {
        double direction = Math.signum(yDesiredDistance - target.getDistanceY()) * motorSpeedMult * -1;
        m_drive.drive(0, direction, 0.0, false);
        //System.out.println("Y:" + direction);
        System.out.println("stage " + "1 " + direction);
    }

    public void alignYaw() {
        double rot = Math.signum(target.getYaw()) * -1;
        rot *= turnSpeedMult;
        m_drive.drive(0.0, 0.0, rot, false);
        //System.out.println("Yaw:" + rot);
        System.out.println("stage " + "2 " + rot);
        //System.out.println("yaw: " + Units.radiansToDegrees(target.getYaw()) );

    }

    public void alignX() {
        double direction = Math.signum(xDesiredDistance - target.getDistanceX()) * motorSpeedMult * -1;
        m_drive.drive(direction, 0.0, 0.0, false);
        //System.out.println("X:" + direction);
        System.out.println("stage " + "3 " + direction);
    }

    public boolean inYMargin() {
        if (Math.abs(yDesiredDistance - target.getDistanceY()) < yDesDistMargin) {
            return true;
        } else {
            return false;
        }
    }

    public boolean inYawMargin() {
        double difference = Math.abs(yawDesiredAngle - target.getYaw());
        difference = Math.min(difference, 360 - difference);
        if (difference < yawDesDistMargin) {
            return true;
        } else {
            return false;
        }
    }

    public boolean inXMargin() {
        //System.out.println("test" + (Math.abs(xDesiredDistance - target.getDistanceX()) < xDesDistMargin));

        if (Math.abs(xDesiredDistance - target.getDistanceX()) < xDesDistMargin) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0,0,0, true);
   
    }
    

}
