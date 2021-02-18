// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.commands.AutoMove;
import frc.commands.AutoTurn;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPath1 extends SequentialCommandGroup {

  public AutoPath1(DriveSubsystem driveSubsystem) {
    addCommands(new AutoMove(driveSubsystem, 7), new AutoTurn(driveSubsystem, 90));
  }
}
