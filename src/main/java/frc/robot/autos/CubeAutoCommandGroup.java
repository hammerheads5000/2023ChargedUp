// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ClawCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperArmToSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CubeAutoCommandGroup extends SequentialCommandGroup {
  /** Creates a new SimpleAutoCommandGroup. */
  public CubeAutoCommandGroup(Swerve s_swerve, UpperArmToSetpoint s_upperArmToSetPoint, LowerArmSubsystem s_lowerArm, ClawCommand cmd_claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // grab w/ claw, drive forward a little
      cmd_claw,
      new InstantCommand(() -> s_swerve.setModuleStates(AutoConstants.driveModuleStates)),
      new WaitCommand(0.5),
      // turn around
      new InstantCommand(() -> s_swerve.setModuleStates(AutoConstants.rotateModuleStates)),
      new WaitCommand(1.0),
      // move arm into position an drive forward a little
      new InstantCommand(() -> s_swerve.setModuleStates(AutoConstants.stopModuleStates)),
      new InstantCommand(() -> s_lowerArm.m_extend()),
      new InstantCommand(() -> s_upperArmToSetPoint.MoveArm(AutoConstants.upparArmPlacementAngle)),
      new InstantCommand(() -> s_swerve.setModuleStates(AutoConstants.driveModuleStates)),
      new WaitCommand(0.5),
      // stop and wait for arm to move
      new InstantCommand(() -> s_swerve.setModuleStates(AutoConstants.stopModuleStates)),
      new WaitCommand(0.5),
      // drop cube
      cmd_claw
    );
  }
}
