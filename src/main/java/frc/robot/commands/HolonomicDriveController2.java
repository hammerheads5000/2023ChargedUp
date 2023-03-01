// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
public class HolonomicDriveController2 {
  private Pose2d m_poseError = new Pose2d();
  private Rotation2d m_rotationError = new Rotation2d();
  private Pose2d m_poseTolerance = new Pose2d();
  private boolean m_enabled = true;

  private final PIDController m_xController;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  private boolean m_firstRun = true;

  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A profiled PID controller to respond to error in angle.
   */
  public HolonomicDriveController2(
      PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
    m_xController = xController;
    m_yController = yController;
    m_thetaController = thetaController;
    m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));
  }

  /**
   * Returns true if the pose error is within tolerance of the reference.
   *
   * @return True if the pose error is within tolerance of the reference.
   */
  public boolean atReference() {
    final var eTranslate = m_poseError.getTranslation();
    final var eRotate = m_rotationError;
    final var tolTranslate = m_poseTolerance.getTranslation();
    final var tolRotate = m_poseTolerance.getRotation();
    return Math.abs(eTranslate.getX()) < tolTranslate.getX()
        && Math.abs(eTranslate.getY()) < tolTranslate.getY()
        && Math.abs(eRotate.getRadians()) < tolRotate.getRadians();
  }

  /**
   * Sets the pose error which is considered tolerance for use with atReference().
   *
   * @param tolerance The pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    m_poseTolerance = tolerance;
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param trajectoryPose The desired trajectory pose, as sampled for the current timestep.
   * @param desiredLinearVelocityMetersPerSecond The desired linear velocity.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d trajectoryPose,
      double desiredLinearVelocityMetersPerSecond,
      Rotation2d desiredHeading) {
    // If this is the first run, then we need to reset the theta controller to the current pose's
    // heading.
    if (m_firstRun) {
      m_thetaController.reset(currentPose.getRotation().getRadians());
      m_firstRun = false;
    }

    SmartDashboard.putNumber("Holonomic_currentPoseX",currentPose.getX());
    SmartDashboard.putNumber("Holonomic_currentPoseY",currentPose.getY());
    SmartDashboard.putNumber("Holonomic_currentPoseRads",currentPose.getRotation().getRadians());
    
    SmartDashboard.putNumber("Holonomic_trajectoryPoseX",trajectoryPose.getX());
    SmartDashboard.putNumber("Holonomic_trajectoryPoseY",trajectoryPose.getY());
    SmartDashboard.putNumber("Holonomic_trajectoryPoseRads",trajectoryPose.getRotation().getRadians());

    SmartDashboard.putNumber("Holonomic_desiredLinearVel",desiredLinearVelocityMetersPerSecond);
    SmartDashboard.putNumber("Holonomic_desiredHeadingRads",desiredHeading.getRadians());

    // Calculate feedforward velocities (field-relative).
    double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
    double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
    double thetaFF =
        m_thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    m_poseError = trajectoryPose.relativeTo(currentPose);
    m_rotationError = desiredHeading.minus(currentPose.getRotation());

    if (!m_enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    }


    

    SmartDashboard.putNumber("PID_xPositionError",m_xController.getPositionError());
    SmartDashboard.putNumber("PID_xPeriod",m_xController.getPeriod());
    SmartDashboard.putNumber("PID_xPositionTolerance",m_xController.getPositionTolerance());
    SmartDashboard.putNumber("PID_xSetpoint",m_xController.getSetpoint());
    SmartDashboard.putNumber("PID_xVelocityError",m_xController.getVelocityError());

    SmartDashboard.putNumber("PID_yPositionError",m_yController.getPositionError());
    SmartDashboard.putNumber("PID_yPeriod",m_yController.getPeriod());
    SmartDashboard.putNumber("PID_yPositionTolerance",m_yController.getPositionTolerance());
    SmartDashboard.putNumber("PID_ySetpoint",m_yController.getSetpoint());
    SmartDashboard.putNumber("PID_yVelocityError",m_yController.getVelocityError());

    SmartDashboard.putNumber("PID_thetaPositionError",m_thetaController.getPositionError());
    SmartDashboard.putNumber("PID_thetaPeriod",m_thetaController.getPeriod());
    SmartDashboard.putNumber("PID_thetaPositionTolerance",m_thetaController.getPositionTolerance());
    SmartDashboard.putString("PID_thetaSetpoint",m_thetaController.getSetpoint().toString());
    SmartDashboard.putString("PID_thetaGoal",m_thetaController.getGoal().toString());


    System.out.println("PID_thetaSetpointPos"+m_thetaController.getSetpoint().position);
    SmartDashboard.putNumber("PID_thetaSetpointPos", m_thetaController.getSetpoint().position);
    System.out.println("PID_thetaGoalPos"+m_thetaController.getGoal().position);
    SmartDashboard.putNumber("PID_thetaGoalPos", m_thetaController.getGoal().position);
    System.out.println("PID_thetaSetpointVel"+m_thetaController.getSetpoint().velocity);
    SmartDashboard.putNumber("PID_thetaSetpointVel", m_thetaController.getSetpoint().velocity);
    System.out.println("PID_thetaGoalVel"+m_thetaController.getGoal().velocity);
    SmartDashboard.putNumber("PID_thetaGoalVel", m_thetaController.getGoal().velocity);


    // Calculate feedback velocities (based on position error).
    double xFeedback = m_xController.calculate(currentPose.getX(), trajectoryPose.getX());
    double yFeedback = m_yController.calculate(currentPose.getY(), trajectoryPose.getY());

    SmartDashboard.putNumber("Holonomic_xFeedback",xFeedback);
    SmartDashboard.putNumber("Holonomic_yFeedback",yFeedback);
    SmartDashboard.putNumber("Holonomic_currentPoseGetRad",currentPose.getRotation().getRadians());
    SmartDashboard.putNumber("Holonomic_xFF",xFF);
    SmartDashboard.putNumber("Holonomic_yFF",yFF);
    SmartDashboard.putNumber("Holonomic_thetaFF",thetaFF);

    //System.out.println("Holonomic_xFF"+xFF);

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF, yFF, 0, currentPose.getRotation());
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param desiredState The desired trajectory pose, as sampled for the current timestep.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading) {
    return calculate(
        currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading);
  }

  /**
   * Enables and disables the controller for troubleshooting problems. When calculate() is called on
   * a disabled controller, only feedforward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    m_enabled = enabled;
  }
}

