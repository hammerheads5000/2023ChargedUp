// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoPathFollowCommand extends CommandBase {
  Pose2d[] poses;
  int targetPoseIndex;
  Swerve s_swerve;
  /** Creates a new AutoPathFollowCommand. */
  public AutoPathFollowCommand(Pose2d[] poses, Swerve s_swerve) {
    this.poses = poses;
    this.targetPoseIndex = 0;
    this.s_swerve = s_swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = poses[targetPoseIndex];
    Pose2d currentPose = s_swerve.swerveOdometry.getPoseMeters();
    Transform2d poseDif = targetPose.minus(currentPose);
    // checks if pose has been reached
    if (Math.abs(poseDif.getX()) < AutoConstants.tolerancePosition 
        && Math.abs(poseDif.getY()) < AutoConstants.tolerancePosition 
        && Math.abs(poseDif.getRotation().getDegrees()) < AutoConstants.toleranceDegrees) { // might need anglemodulus
      targetPoseIndex++;
    }
    // will go to next pose if the end of poses was not reached
    if (targetPoseIndex < poses.length) {
      targetPose = poses[targetPoseIndex];
    }
    // get direction to go in and scale it to speed determined in constants
    double[] velocity = scaledToLength(new double[]{targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY()}, AutoConstants.maxDriveSpeed);
    double angularVelocity = MathUtil.angleModulus(targetPose.getRotation().minus(currentPose.getRotation()).getRadians()); // angle difference
    angularVelocity = Math.signum(angularVelocity) * AutoConstants.maxAngularVelocityRadians;
    s_swerve.drive(velocity[0], velocity[1], angularVelocity, true); // DRIVE
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetPoseIndex >= poses.length; // returns true when there are no poses left
  }

  // scales vector to desired length
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
