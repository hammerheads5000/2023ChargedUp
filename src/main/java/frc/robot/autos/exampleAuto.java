package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.AutoSwerve;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(AutoSwerve s_AutoSwerve){
        
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("maxPath", new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        
        // This is just an example event map. It would be better to have a constant, global event map - in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        
        FollowPathWithEvents PPswerveControllerCommand =
        new FollowPathWithEvents(
            new PPSwerveControllerCommand(
                examplePath,
                s_AutoSwerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                new PIDController(0, 0, 0), // Rotation controller
                s_AutoSwerve::setModuleStates,
                true,
                s_AutoSwerve),
            examplePath.getMarkers(),
            eventMap);        


        addCommands(
            new InstantCommand(() -> s_AutoSwerve.resetOdometry(examplePath.getInitialPose())),
            //PPswerveControllerCommand, //commented out but maybe shouldn't be
            new InstantCommand(() -> s_AutoSwerve.drive(1,0,0,false))       // Need to stop robot at end, likely different param to set FieldRelative
        );
    }
}