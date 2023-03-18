// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.time.Instant;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoPathFollowCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPathFollow extends SequentialCommandGroup {
  /** Creates a new TestPathFollow. */
  public TestPathFollow(Swerve s_swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerTrajectory traj = 
        PathPlanner.loadPath("Test Path", new PathConstraints(AutoConstants.maxVel, AutoConstants.maxAcc));
    PPSwerveControllerCommand swerveCommand = new PPSwerveControllerCommand(
      traj, 
      s_swerve::getPose, 
      Constants.Swerve.swerveKinematics,
      new PIDController(0, 0, 0), 
      new PIDController(0, 0, 0), 
      new PIDController(0, 0, 0), 
      s_swerve::setModuleStates,
      true,
      s_swerve
    );

    addCommands(
      swerveCommand
    );
  }
}
