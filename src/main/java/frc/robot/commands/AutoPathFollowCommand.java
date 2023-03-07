// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoPathFollowCommand extends CommandBase {
  Pose2d[] poses;
  Pose2d targetPose;
  Swerve s_swerve;
  /** Creates a new AutoPathFollowCommand. */
  public AutoPathFollowCommand(Pose2d[] poses, Swerve s_swerve) {
    this.poses = poses;
    this.targetPose = poses[1];
    this.s_swerve = s_swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_swerve.resetOdometry(poses[0]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    Pose2d currentPose = s_swerve.swerveOdometry.getPoseMeters();
    double[] velocity = scaledToLength(new double[]{targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY()}, AutoConstants.maxDriveSpeed);
    double angularVelocity = MathUtil.angleModulus(targetPose.getRotation().minus(currentPose.getRotation()).getRadians()); // angle difference
    angularVelocity = Math.signum(angularVelocity) * AutoConstants.maxAngularVelocityRadians;
    s_swerve.drive(velocity[0], velocity[1], angularVelocity, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // scales vector to length
  public double[] scaledToLength(double[] vector, double length) {
    double scale = 0;
    for (double val : vector) {
      scale += val*val;
    }
    scale = length/Math.sqrt(scale);
    double[] result = new double[vector.length];
    for (int i = 0; i < result.length; i++) {
      result[i] = vector[i] * scale;
    }

    return result;
  }
}