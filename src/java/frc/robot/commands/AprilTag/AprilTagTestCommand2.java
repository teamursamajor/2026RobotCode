// package frc.robot.commands.AprilTag;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.AprilTag.AprilTagAlign;
// import frc.robot.subsystems.AprilTag.AprilTagSubsystem;
// import frc.robot.subsystems.Drive.DriveSubsystem;

// public class AprilTagTestCommand2 extends Command {



//     AprilTagSubsystem aprilTag = new AprilTagSubsystem();

//     DriveSubsystem m_drive;

//     public AprilTagTestCommand2(DriveSubsystem m_drive, AprilTagSubsystem apriltag){
//         this.aprilTag = apriltag;
//         this.m_drive = m_drive;
//     }

//     private double yDesiredDistance = 0;
//     private double xDesiredDistance = 0.5;
//     private double yawDesired = 0;

//     private double yDesDistMargin = 0.1;
//     private double xDesDistMargin = 0.1;
//     private double yawDesDistMargin = 1;


//     @Override
//     public void execute() {
        
//         AprilTagAlign target = aprilTag.targetValues();
//         SmartDashboard.putNumber("Y value", target.getDistanceY());
//         System.out.println(target.getDistanceY());
        
//         // Shuffleboard.getTab("testWindow").add("distance", target.getDistance());

//         if (target.getId() != Double.MAX_VALUE) {
//             if (checkstage(target) == 0) {
//                 alignY(target);
//             }
//             if (checkstage(target) == 1) {
//                 alignYaw(target);
//             }
//             if (checkstage(target) == 2) {
//                 alignX(target);
//             }
//             if (checkstage(target) == 3) {
//                 isFinished();
//             }
//         }

//     }

//     public int checkstage(AprilTagAlign target) {

//         int alignStage = 3;
//         if (inXMargin(target) == false) {
//             alignStage = 2;
//         }
//         if (inYawMargin(target) == false) {
//             alignStage = 1;
//         }
//         if (inYMargin(target) == false) {
//             alignStage = 0;
//         }

//         return alignStage;
//     }

//     public boolean inYMargin(AprilTagAlign target) {
//         if (Math.abs(yDesiredDistance - target.getDistanceY()) < yDesDistMargin) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     public boolean inYawMargin(AprilTagAlign target) {
//         double difference = Math.abs(yawDesired - target.getYaw());
//         difference = Math.min(difference, 360 - difference);
//         if (difference < yawDesDistMargin) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     public boolean inXMargin(AprilTagAlign target) {
//         if (Math.abs(xDesiredDistance - target.getDistanceX()) < xDesDistMargin) {
//             return true;
//         } else {
//             return false;
//         }
//     }

//     public void alignY(AprilTagAlign target) {
//         double direction = Math.signum(yDesiredDistance - target.getDistanceY()) * 0.5;
//         m_drive.drive(0, direction, 0.0, false);
//     }

//     public void alignYaw(AprilTagAlign target) {

//         double rot = Math.signum(target.getYaw()) * -1;
//         rot *= 0.5;
//         m_drive.drive(0.0, 0.0, rot, false);
//     }

//     public void alignX(AprilTagAlign target) {
//         double direction = Math.signum(xDesiredDistance - target.getDistanceX()) * 0.5;
//         m_drive.drive(direction, 0.0, 0.0, false);
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         m_drive.drive(0, 0, 0, false);
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
