package frc.robot.autos;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SwerveControllerCommand2;
import frc.robot.subsystems.Swerve;


public class manualAuto extends SequentialCommandGroup {
    Trajectory exampleTrajectory = new Trajectory();

    public manualAuto(Swerve s_AutoSwerve, String path){

        TrajectoryConfig config =

    new TrajectoryConfig(

            AutoConstants.kMaxSpeedMetersPerSecond,

            AutoConstants.kMaxAccelerationMetersPerSecondSquared)

        // Add kinematics to ensure max speed is actually obeyed

        .setKinematics(Constants.Swerve.swerveKinematics);

    String trajectoryJSON = path;
try{
Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
}
catch (IOException ex){
    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());

}


/*TrajectoryGenerator.generateTrajectory(

    // Start at the origin facing the +X direction

    new Pose2d(0, 0, new Rotation2d(0)),

    // Pass through these two interior waypoints

    List.of(new Translation2d(.3, .3), new Translation2d(.7, .7)),

    // End straight ahead of where we started, facing forward

    new Pose2d(1, 1, new Rotation2d(0)),

    config);*/
    var thetaController =

    new ProfiledPIDController(

        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

thetaController.enableContinuousInput(-Math.PI, Math.PI);

SwerveControllerCommand2 swerveControllerCommand2 =

new SwerveControllerCommand2(

 

exampleTrajectory,



s_AutoSwerve::getPose, // Functional interface to feed supplier



Constants.Swerve.swerveKinematics,



        // Position controllers

        new PIDController(AutoConstants.kPXController, 0.0, 0.0),

        new PIDController(AutoConstants.kPYController, 0.0, 0.0),

        thetaController,

        s_AutoSwerve::setModuleStates,

        s_AutoSwerve);



//and use this code instead of what it shows so you can put it in same exampleAuto.java file:



    addRequirements(s_AutoSwerve);


    
    addCommands(

        
        new InstantCommand(() -> s_AutoSwerve.resetOdometry(exampleTrajectory.getInitialPose())),

       //swerveControllerCommand2,
        new InstantCommand(() -> s_AutoSwerve.drive(1,0,0,false)),       // Need to stop robot at end, likely different param to set FieldRelative
        new WaitCommand(1.0),
        new InstantCommand(() -> s_AutoSwerve.drive(0,0,0,false))       // Need to stop robot at end, likely different param to set FieldRelative


    );


}




    

    }


