package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class LeaveToDirection extends Command{
      private DriveSubsystem m_drive;

    public LeaveToDirection(DriveSubsystem driveSubsystem){
        addRequirements(driveSubsystem);
        m_drive = driveSubsystem;
    }
    @Override
    public void initialize() {
        m_drive.drive(1, 0, 0, isFinished());
    }
}
