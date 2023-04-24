// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Arm_Commands.ArmPresetCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GamePieceAuto extends SequentialCommandGroup {
  final IntegerEntry placedEntry; // entry to interact with network tables

  /** Creates a new GamePiece. */
  public GamePieceAuto(
    Swerve s_swerve, 
    ArmPresetCommand cmd_armExtend, 
    ArmPresetCommand cmd_armStow, 
    ArmPresetCommand cmd_armPortal, 
    ClawSubsystem sub_clawSubsystem, 
    LowerArmSubsystem sub_lowerArm,
    String cyclePath, String placePath) {

    placedEntry = NetworkTableInstance.getDefault().getIntegerTopic("/SmartDashboard/piecesPlaced").getEntry(0); // entry for the number of pieces placed
    
    // maps event names in pathplanner to commands
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Drop", new InstantCommand(() -> sub_clawSubsystem.m_extend()));
    eventMap.put("Grab", new InstantCommand(() -> sub_clawSubsystem.m_contract()));
    eventMap.put("Extend arm", cmd_armExtend);
    eventMap.put("Arm back", cmd_armStow);
    eventMap.put("Game piece", this); // will restart cycle after it finishes

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      s_swerve::getPose, // Pose2d supplier
      s_swerve::resetOdometry, // Pose2d consumer to reset odometry
      Constants.Swerve.swerveKinematics, // SwerveDriveKinematics
      Constants.AutoConstants.drivePID, // PID constants for translation
      Constants.AutoConstants.anglePID, // PID constants for rotation
      s_swerve::setModuleStates, // Module states consumer used to drive
      eventMap, // Mapping of Strings to Commands
      s_swerve // Drive subsystem
    );

    // configures going to and from portal
    PathPlannerTrajectory cycleTraj = PathPlanner.loadPath(cyclePath, new PathConstraints(AutoConstants.maxVel, AutoConstants.maxAcc));
    Command cycleCommand = autoBuilder.fullAuto(cycleTraj);

    // configures placing of cube/cone command
    PathPlannerTrajectory placeTraj = PathPlanner.loadPath(placePath, new PathConstraints(AutoConstants.maxVel, AutoConstants.maxAcc));
    Command placeCommand = autoBuilder.fullAuto(placeTraj);
    
    Command transformPlacePath = new InstantCommand(() -> {
      placeTraj.transformBy(new Transform2d(placeTraj.getInitialPose(), s_swerve.getPose()));
      autoBuilder.fullAuto(placeTraj);
    }); // updates place path to be where the robot is
    
    // manually generated trajectories
    PathConstraints constraints = new PathConstraints(Constants.AutoConstants.maxVel, Constants.AutoConstants.maxAcc);
    
    // going to the game piece
    PathPlannerTrajectory toTrajectory = PathPlanner.generatePath(
      constraints, 
      new PathPoint(s_swerve.getPose().getTranslation(), s_swerve.getPose().getRotation(), Rotation2d.fromDegrees(90)), // initial point, end of cycle
      new PathPoint(placeTraj.getInitialPose().getTranslation(), placeTraj.getInitialPose().getRotation(), placeTraj.getInitialHolonomicPose().getRotation()) // endpoint, start of placing
    );
    Command toCommand = autoBuilder.fullAuto(toTrajectory);
    
    // going from the game piece to the start of the cycle
    PathPlannerTrajectory fromTrajectory = PathPlanner.generatePath(
      constraints,
      new PathPoint(placeTraj.getEndState().poseMeters.getTranslation(), placeTraj.getEndState().poseMeters.getRotation()), // initial point, end of placing
      new PathPoint(cycleTraj.getInitialPose().getTranslation(), cycleTraj.getInitialPose().getRotation(), cycleTraj.getInitialHolonomicPose().getRotation()) // endpoint, start of cycle
    );
    Command fromCommand = autoBuilder.fullAuto(fromTrajectory);

    addCommands(
      toCommand,
      transformPlacePath,
      placeCommand,
      new InstantCommand(() -> updatePlaced()),
      fromCommand,
      cycleCommand
    );
  }

  private void updatePlaced() {
    placedEntry.set(placedEntry.get()+1);
  }
}
