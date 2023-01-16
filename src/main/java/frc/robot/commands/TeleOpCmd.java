// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsytem;

public class TeleOpCmd extends CommandBase {

  private final DriveSubsytem driveSubsytem;
  private final Supplier<Double> xSupplier, ySupplier, zSupplier;

  /** Creates a new TeleOpCmd. */
  public TeleOpCmd(DriveSubsytem subsytem, Supplier<Double> xSupplier, Supplier<Double> ySupplier, Supplier<Double> zSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsytem = subsytem;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.zSupplier = zSupplier;
    addRequirements(subsytem);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = xSupplier.get();
    double ySpeed = ySupplier.get();
    double zSpeed = ySupplier.get();

    driveSubsytem.drive(xSpeed, ySpeed, zSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsytem.StopModule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
