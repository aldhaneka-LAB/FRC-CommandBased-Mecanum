package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExampleSubsystem;

public class ExampleCommand extends CommandBase {
    public ExampleCommand(ExampleSubsystem subsystem) {
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Initialise example commadn");
    }

    @Override
    public void execute() {
        System.out.println("Execure example command");
        
    }
    // public

    @Override
    public void end(boolean interrupted) {
        System.out.println("end example commadn!");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
