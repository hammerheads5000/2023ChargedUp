// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.Arm_Commands.ArmPresetCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathAuto extends SequentialCommandGroup {
  /** Creates a new PathAuto. */
  public PathAuto(Swerve s_swerve, ArmPresetCommand cmd_armExtend, ArmPresetCommand cmd_armStow, LowerArmSubsystem s_lowerArm, ClawSubsystem sub_ClawSubsystem, Command cmd_autoBalance, String path) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    List<PathPlannerTrajectory> pathGroup = 
        PathPlanner.loadPathGroup(path, new PathConstraints(AutoConstants.maxVel, AutoConstants.maxAcc));
    
    // hashmap mapping strings set in PathPlanner to commands to execute
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("Auto balance", cmd_autoBalance);
    eventMap.put("Drop", new InstantCommand(() -> sub_ClawSubsystem.m_extend()));
    eventMap.put("Extend arm", cmd_armExtend);
    eventMap.put("Arm back", cmd_armStow);

    // Will make auto command automatically
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
    
    // Build command automatically
    Command autoCommand = autoBuilder.fullAuto(pathGroup);

    addCommands(
      autoCommand
    );
  }
}
