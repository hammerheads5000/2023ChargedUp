// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    this.targetPoseIndex = 1;
    this.s_swerve = s_swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_swerve.resetOdometry(poses[1]);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d targetPose = poses[targetPoseIndex];
    Pose2d currentPose = s_swerve.swerveOdometry.getPoseMeters();
    Transform2d poseDif = targetPose.minus(currentPose);
    // checks if pose has been reached
    if (isNegligable(poseDif)) { // might need anglemodulus
      targetPoseIndex++;
    }
    // will go to next pose if the end of poses was not reached
    if (targetPoseIndex < poses.length) {
      targetPose = poses[targetPoseIndex];
    }
    // calculate positional velocity
    Translation2d velocity = poseDif.getTranslation();
    velocity = velocity.div(velocity.getNorm()); // set to unit vector
    double proportionDriven = 
          getProportionDriven(poses[targetPoseIndex-1], targetPose, currentPose);
    double speed = getSpeedFromProportionDriven(
          proportionDriven, 
          AutoConstants.minDriveSpeed, 
          AutoConstants.maxDriveSpeed);
    
    velocity = velocity.times(speed); // set to speed

    // calculate angular velocity
    double angularVelocity = targetPose.getRotation().minus(currentPose.getRotation()).getRadians(); // angle difference
    angularVelocity = MathUtil.angleModulus(angularVelocity); // mod the angle
    double proportionRotated = 
          getProportionRotated(poses[targetPoseIndex-1], targetPose, currentPose);
    double velocityMultiplier = getAngularVelocityFromProportionRotated(
          proportionRotated, 
          AutoConstants.minAngularVelocityRadians, 
          AutoConstants.maxAngularVelocityRadians);

    angularVelocity = Math.signum(angularVelocity) * velocityMultiplier; // set velocity
    
    s_swerve.drive(velocity.getX(), velocity.getY(), angularVelocity, true); // DRIVE
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_swerve.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetPoseIndex >= poses.length; // returns true when there are no poses left
  }

  private boolean isNegligable(Transform2d transform) {
    boolean xNegligable = Math.abs(transform.getX()) < AutoConstants.tolerancePosition;
    boolean yNegligable = Math.abs(transform.getY()) < AutoConstants.tolerancePosition;
    boolean angleNegligable = Math.abs(transform.getRotation().getDegrees()) < AutoConstants.toleranceDegrees;
    return xNegligable && yNegligable && angleNegligable;
  }

  // gets rough proportion of path between 2 points driven
  private double getProportionDriven(Pose2d start, Pose2d end, Pose2d current) {
    double distanceDriven = current.minus(start).getTranslation().getNorm();
    double totalDistance = end.minus(start).getTranslation().getNorm();
    return distanceDriven / totalDistance;
  }

  // gets rough proportion of rotation between 2 points rotated
  private double getProportionRotated(Pose2d start, Pose2d end, Pose2d current) {
    double distanceRotated = current.minus(start).getRotation().getRadians();
    double totalDist = end.minus(start).getRotation().getRadians();
    distanceRotated = MathUtil.angleModulus(distanceRotated);
    totalDist = MathUtil.angleModulus(totalDist);
    return distanceRotated / totalDist;
  }

  // returns speed to drive at given a certain proportion has already been driven
  private double getSpeedFromProportionDriven(double x, double min, double max) {
    double speed =  Math.cos(Math.PI*x - Math.PI/2);
    speed *= (max - min);
    speed += min;
    return speed;
  }

  // returns angular velocity to drive at given a certain proportion has already been rotated
  private double getAngularVelocityFromProportionRotated(double x, double min, double max) {
    double angularVelocity =  Math.cos(Math.PI*x - Math.PI/2);
    angularVelocity *= (max - min);
    angularVelocity += min;
    return angularVelocity;
  }
}
