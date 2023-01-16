package frc.robot.commands;

import javax.swing.GroupLayout.SequentialGroup;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.DriveSubsystem.MoveFront;
import frc.robot.commands.DriveSubsystem.StopRobot;
import frc.robot.subsystems.DriveSubsytem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class CommandAuto extends SequentialCommandGroup {
    public CommandAuto(DriveSubsytem drive) {
        // CommandBase

        // Timer
        // Commands
        super(
            new MoveFront(drive, 2, 0.1),
            new StopRobot(drive)
            // new WaitCommand(2),
            // new ParallelRaceGroup(new MoveFront(drive, 2), new WaitCommand(2)),
            // new StopRobot(drive)
        );
        
    }   
}
