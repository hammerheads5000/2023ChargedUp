// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BalanceAutoCommandGroup extends SequentialCommandGroup {
  /** Creates a new SimpleAutoCommandGroup. */
  public BalanceAutoCommandGroup(Swerve s_swerve, AutoBalanceCommand cmd_autoBalance) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> s_swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      // drive forward @ 2 m/s for 2 seconds then stop
      new InstantCommand(() -> s_swerve.drive(2.0, 0.0, 0.0, false)),
      new WaitCommand(2.0),
      cmd_autoBalance
    );
  }
}
