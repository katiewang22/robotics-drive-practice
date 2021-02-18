// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoMove extends CommandBase {
  
  private final DriveSubsystem driveSubsystem;
  private double start_distance_meters;
  private double target_distance;

  public AutoMove(DriveSubsystem subsystem, double distance_in_meters) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = subsystem;
    addRequirements(subsystem);
    start_distance_meters = driveSubsystem.distanceTravelledInMeters();
    target_distance = distance_in_meters;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetEncoders();
    driveSubsystem.zeroYaw();
    start_distance_meters = driveSubsystem.distanceTravelledInMeters();
    driveSubsystem.state_flag_motion_profile = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.drive(target_distance, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceDriven = driveSubsystem.distanceTravelledInMeters() - start_distance_meters;
    double positionError = Math.abs(target_distance - distanceDriven);

    return (positionError < 0.2);
  }
}
