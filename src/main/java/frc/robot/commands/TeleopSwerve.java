package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    public boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick drive;
    private Joystick manip;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Joystick drive, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.drive = drive;
        this.manip = manip;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void initialize(){s_Swerve.zeroGyro();}

    @Override
    public void execute() {
        //fieldRelative = !drive.getRawButton(XboxController.Button.kLeftBumper.value);

        double d_yAxis = -drive.getRawAxis(translationAxis);
        double d_xAxis = -drive.getRawAxis(strafeAxis);
        double d_rAxis = -drive.getRawAxis(rotationAxis);

        d_yAxis = MathUtil.clamp(d_yAxis, -1, 1); 
        d_xAxis = MathUtil.clamp(d_xAxis, -1, 1);
        
        /* Deadbands */
        d_yAxis = (Math.abs(d_yAxis) < Constants.stickDeadband) ? 0 : d_yAxis;
        d_xAxis = (Math.abs(d_xAxis) < Constants.stickDeadband) ? 0 : d_xAxis;
        d_rAxis = (Math.abs(d_rAxis) < Constants.stickDeadband) ? 0 : d_rAxis;

        double multiplier = 0.7;
        translation = new Translation2d(Math.signum(d_yAxis) * d_yAxis*d_yAxis * multiplier, Math.signum(d_xAxis) * d_xAxis*d_xAxis * multiplier).times(Constants.Swerve.maxSpeed);
        rotation = d_rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
        SmartDashboard.updateValues();

    }

    
}