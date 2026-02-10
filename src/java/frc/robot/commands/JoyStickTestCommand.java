
package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class JoyStickTestCommand extends Command {
    int count = 0;

    @Override
    public void initialize() {

    }

    public JoyStickTestCommand() {

    }

    @Override
    public void execute() {
        count++;

        System.out.println("something2");
    }

    @Override
    public void end(boolean interrupted) {

    }

}