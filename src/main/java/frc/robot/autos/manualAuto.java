package frc.robot.autos;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;


public class manualAuto extends SequentialCommandGroup {
    public manualAuto(Swerve s_AutoSwerve){

        TrajectoryConfig config =

    new TrajectoryConfig(

            AutoConstants.kMaxSpeedMetersPerSecond,

            AutoConstants.kMaxAccelerationMetersPerSecondSquared)

        // Add kinematics to ensure max speed is actually obeyed

        .setKinematics(Constants.Swerve.swerveKinematics);


Trajectory exampleTrajectory =




TrajectoryGenerator.generateTrajectory(

    // Start at the origin facing the +X direction

    new Pose2d(0, 0, new Rotation2d(0)),

    // Pass through these two interior waypoints

    List.of(new Translation2d(.3, 0), new Translation2d(.7, 0)),

    // End straight ahead of where we started, facing forward

    new Pose2d(1, 0, new Rotation2d(0)),

    config);
    var thetaController =

    new ProfiledPIDController(

        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

thetaController.enableContinuousInput(-Math.PI, Math.PI);



SwerveControllerCommand swerveControllerCommand =

    new SwerveControllerCommand(

        exampleTrajectory,

        s_AutoSwerve::getPose, // Functional interface to feed supplier

        Constants.Swerve.swerveKinematics,



        // Position controllers

        new PIDController(AutoConstants.kPXController, 0, 0),

        new PIDController(AutoConstants.kPYController, 0, 0),

        thetaController,

        s_AutoSwerve::setModuleStates,

        s_AutoSwerve);



//and use this code instead of what it shows so you can put it in same exampleAuto.java file:



    addRequirements(s_AutoSwerve);



    addCommands(

        new InstantCommand(() -> s_AutoSwerve.resetOdometry(exampleTrajectory.getInitialPose())),

       swerveControllerCommand,

        new InstantCommand(() -> s_AutoSwerve.drive(0,0,0,false))       // Need to stop robot at end, likely different param to set FieldRelative

    );


}




    

    }


