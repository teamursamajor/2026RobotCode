package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class LeaveLeftCommand extends Command {
    private DriveSubsystem m_drive;
    private boolean isFinished = false;
    private long time;

    public LeaveLeftCommand(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);
        m_drive = driveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time = RobotController.getFPGATime();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        long currentTime = RobotController.getFPGATime() - time;
        // Place Holder Values
        m_drive.drive(0, 0, 60, false);
        if (currentTime > (2 * 1000000)) {
            m_drive.drive(0.5, 0, 0, false);
        }
        if (currentTime > (2.5 * 1000000)) {
            m_drive.drive(0, 0, -30, false);
        }
        if (currentTime > (3 * 1000000)) {
            m_drive.drive(0.5, 0, 0, false);
        }
        if (currentTime > (4 * 1000000)) {
            m_drive.drive(0.5, 0, 0, false);
        }
        if (currentTime > 4.5 * 1000000) {
            isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0.5, 0, 0, false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;

    }
}
