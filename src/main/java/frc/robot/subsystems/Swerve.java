package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.sql.Driver;

import com.ctre.phoenix.sensors.PigeonIMU;
//import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.SwerveModule;
import frc.lib.math.Conversions;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public WPI_Pigeon2 gyro;
    private Joystick drive;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private boolean fieldRelative;
    private boolean openLoop;
    //public Field2d m_field2d = new Field2d();

    public Swerve(Joystick drive, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.drive = drive;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        
        gyro = new WPI_Pigeon2(Constants.Swerve.pigeonID, "Bobby");
        zeroGyro();
        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        zeroWheels();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }


  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    

    var swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.Swerve.maxSpeed);
    mSwerveMods[0].setDesiredState(swerveModuleStates[0], true);
    mSwerveMods[1].setDesiredState(swerveModuleStates[1], true);
    mSwerveMods[2].setDesiredState(swerveModuleStates[2], true);
    mSwerveMods[3].setDesiredState(swerveModuleStates[3], true);
  }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }  
    
    public SwerveModulePosition[] getModulePositions()
    {
        SmartDashboard.putString("getPosition[0]",mSwerveMods[0].getPosition().toString());
        SmartDashboard.putString("getPosition[1]",mSwerveMods[1].getPosition().toString());
        SmartDashboard.putString("getPosition[2]",mSwerveMods[2].getPosition().toString());
        SmartDashboard.putString("getPosition[3]",mSwerveMods[3].getPosition().toString());
        return new SwerveModulePosition[] {
            mSwerveMods[0].getPosition(),
            mSwerveMods[1].getPosition(),
            mSwerveMods[2].getPosition(),
            mSwerveMods[3].getPosition()
            
        };
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], true);//true?
        }
    }    
    
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
        gyro.reset();
    }

    public void zeroWheels(){
        for (SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Rotation2d getYaw() {
        double yaw = -gyro.getYaw();
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }
    
    public Trajectory loadTrajectoryFromFile(String filename) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
            return TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + filename, e.getStackTrace());
            return new Trajectory();
        }
    }

    @Override
    public void periodic(){
        // // SmartDashboard.putNumber("getXpreUpdt",swerveOdometry.getPoseMeters().getX());
        // // SmartDashboard.putNumber("getYpreUpdt",swerveOdometry.getPoseMeters().getY());
        // // SmartDashboard.putNumber("getGyroAnglepreUpdt",gyro.getAngle());
        swerveOdometry.update(getYaw(), getModulePositions());  
        // // SmartDashboard.putNumber("getXpostUpdt",swerveOdometry.getPoseMeters().getX());
        // // SmartDashboard.putNumber("getYpostUpdt",swerveOdometry.getPoseMeters().getY());
        // SmartDashboard.putNumber("getGyroAnglepostUpdt",gyro.getAngle());
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    

            SmartDashboard.putNumber("Mod " + mod.moduleNumber + "Talon Angle Motor", mod.getTalonAngleMotor());
        }
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());

        // double d_yAxis = -drive.getRawAxis(translationAxis);
        // double d_xAxis = -drive.getRawAxis(strafeAxis);
        // double d_rAxis = -drive.getRawAxis(rotationAxis);

        // d_yAxis = MathUtil.clamp(d_yAxis, -1, 1); 
        // d_xAxis = MathUtil.clamp(d_xAxis, -1, 1);
        
        // /* Deadbands */
        // d_yAxis = (Math.abs(d_yAxis) < Constants.stickDeadband) ? 0 : d_yAxis;
        // d_xAxis = (Math.abs(d_xAxis) < Constants.stickDeadband) ? 0 : d_xAxis;
        // d_rAxis = (Math.abs(d_rAxis) < Constants.stickDeadband) ? 0 : d_rAxis;

        // double multiplier = 0.7;
        // Translation2d translation = new Translation2d(Math.signum(d_yAxis) * d_yAxis*d_yAxis * multiplier, Math.signum(d_xAxis) * d_xAxis*d_xAxis * multiplier).times(Constants.Swerve.maxSpeed);
        // double rotation = d_rAxis * Constants.Swerve.maxAngularVelocity;
        // drive(translation, rotation, fieldRelative, openLoop);
        // SmartDashboard.updateValues();
        
    
    }
    
    /*public void zeroWheels()
    {
        for(SwerveModule modZero : mSwerveMods)
        {
            while(Math.abs((modZero.getAngle()-modZero.angleOffset)>0.1)
            {
                SwerveModuleState desired = new SwerveModuleState(0);
                if(modZero.getAngle()-modZero.angleOffset>0)
                {
                    modZero.setDesiredState(desired,false);
                }
                if(modZero.getAngle()-modZero.angleOffset)
                {
                    modZero.setDesiredState(desired,false);
                }
            }
        }*/
        /*public void zeroWheels()
        {
            for(SwerveModule modZero : mSwerveMods)
            {
                modZero.resetToAbsolute();
            }
        }*/
    }
    

