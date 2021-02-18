// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class JoystickDrive extends CommandBase {
  
    private final DriveSubsystem driveSubsystem;
    private static XboxController driveController = RobotContainer.driverController;

    public JoystickDrive(DriveSubsystem drivetrain) {
        driveSubsystem = drivetrain;
        addRequirements(driveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double throttle = driveController.getY(Hand.kLeft);
        double rotate = driveController.getX(Hand.kRight);

        // Deadbands
        if ((throttle > 0  && throttle < 0.25) || (throttle < 0 && throttle > -0.25)) {
            throttle = 0;
        } else {
            throttle = throttle;
        }

        if ((rotate > 0  && rotate < 0.25) || (rotate < 0 && rotate > -0.25)) {
            rotate = 0;
        } else {
            rotate = rotate;
        }
        
        // Scale rotate
        rotate = rotate * 2;

        if (driveController.getTriggerAxis(Hand.kRight) > 0.25) {
            throttle = Math.signum(throttle) * 0.75;
        } else if (driveController.getAButton()) {
            throttle = (throttle * 1.1);
        } else {
            throttle = (throttle * 0.8);
        }

        driveSubsystem.drive(throttle, rotate);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
