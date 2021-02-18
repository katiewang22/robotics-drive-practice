package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.commands.JoystickDrive;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    public static DriveSubsystem driveSubsystem = new DriveSubsystem();
    public static XboxController driverController = new XboxController(0);

    public RobotContainer() {
        configureButtonBindings();
        driveSubsystem.setDefaultCommand(new JoystickDrive(driveSubsystem));
    }

    private void configureButtonBindings() {

    }
}