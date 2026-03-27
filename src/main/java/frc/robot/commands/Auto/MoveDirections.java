package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class MoveDirections extends Command {
    double xSpeed;
    double ySpeed;
    double rotationSpeed;
    private DriveSubsystem m_drive;

    public MoveDirections(DriveSubsystem driveSubsystem, double xSpeed, double ySpeed,double rotationSpeed) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotationSpeed = rotationSpeed;
        addRequirements(driveSubsystem);
        m_drive = driveSubsystem;   
    }

    @Override
    public void initialize() {
        m_drive.drive(xSpeed, ySpeed, rotationSpeed, true);
    }
    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }
}
