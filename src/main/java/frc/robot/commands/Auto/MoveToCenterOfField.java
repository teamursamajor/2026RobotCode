package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class MoveToCenterOfField extends Command{
    private DriveSubsystem m_drive;
    private String leaveDirection;

    public MoveToCenterOfField(DriveSubsystem driveSubsystem, String leaveDirection){
        addRequirements(driveSubsystem);
        m_drive = driveSubsystem;
    }
    @Override
    public void initialize() {
        if(leaveDirection == "Left"){
            new MoveDirections(m_drive, 0, -1).withTimeout(2)
            .andThen(new MoveDirections(m_drive, 1, 0).withTimeout(2))
            .andThen(new MoveDirections(m_drive, 0, 1).withTimeout(1));
        }
        if(leaveDirection == "Right"){
            new MoveDirections(m_drive, 0, 1).withTimeout(2)
            .andThen(new MoveDirections(m_drive, 1, 0).withTimeout(2))
            .andThen(new MoveDirections(m_drive, 0, -1).withTimeout(1));
        }
        if(leaveDirection == "BackThenRight"){
            new MoveDirections(m_drive, -1, 0).withTimeout(1)
            .andThen(new MoveDirections(m_drive, 0, 1).withTimeout(2))
            .andThen(new MoveDirections(m_drive, 1, 0).withTimeout(2))
            .andThen(new MoveDirections(m_drive, 0, -1).withTimeout(1));
        }
    }
}
