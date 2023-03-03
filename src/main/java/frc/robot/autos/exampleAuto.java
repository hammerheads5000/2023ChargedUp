package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoArmSet;
import frc.robot.commands.ClawCommand;
import frc.robot.commands.LowerArmCommand;
import frc.robot.commands.PPSwerveControllerCommand2;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.LowerArmSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class exampleAuto extends SequentialCommandGroup {
    public exampleAuto(Swerve s_AutoSwerve, String path, LowerArmSubsystem sub_LowerArmSubsystem, int angle, ClawSubsystem sub_ClawSubsystem){
        
        PathPlannerTrajectory examplePath = PathPlanner.loadPath(path, new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        // This is just an example event map. It would be better to have a constant, global event map - in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("lowerArm", new LowerArmCommand(sub_LowerArmSubsystem));
        eventMap.put("claw", new ClawCommand(sub_ClawSubsystem));


        
    
   

        

        var thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        FollowPathWithEvents PPswerveControllerCommand2 =
        new FollowPathWithEvents(
            new PPSwerveControllerCommand2(
                examplePath,
                s_AutoSwerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_AutoSwerve::setModuleStates,
                false,
                s_AutoSwerve),
            examplePath.getMarkers(),
            eventMap);        


        addCommands(
            new InstantCommand(() -> s_AutoSwerve.zeroGyro()),
            new InstantCommand(() -> s_AutoSwerve.resetOdometry(examplePath.getInitialPose())),
            PPswerveControllerCommand2, //commented out but maybe shouldn't be
            //new InstantCommand(() -> s_AutoSwerve.drive(new Translation2d(1, 0),0,false, true)),       // Need to stop robot at end, likely different param to set FieldRelative
            //new WaitCommand(2),
            new InstantCommand(() -> s_AutoSwerve.drive(new Translation2d(0, 0),0,false, true))      // Need to stop robot at end, likely different param to set FieldRelative

            

        );
    }
}