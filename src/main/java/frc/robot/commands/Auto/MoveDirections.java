package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class MoveDirections extends Command{
    double xSpeed;
    double ySpeed;

    private DriveSubsystem m_drive;

    public MoveDirections(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed){
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        addRequirements(driveSubsystem);
        m_drive = driveSubsystem;
    }
    @Override
    public void initialize() {
        m_drive.drive(xSpeed, ySpeed, 0, true);
    }
}
