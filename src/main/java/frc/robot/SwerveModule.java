package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.DutyCycleOut;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.controls.PositionDutyCycle;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.hardware.CANcoder;

public class SwerveModule {
    public int moduleNumber;
    private double angleOffset;
    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;
    private double lastAngle;
    public SwerveModuleConstants moduleConstants;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleConstants = moduleConstants;
        this.moduleNumber = moduleNumber;
        angleOffset = moduleConstants.angleOffset;
        
             /* Angle Encoder Config */
            angleEncoder = new CANcoder(moduleConstants.cancoderID, "Bobby");
            configAngleEncoder();

             /* Angle Motor Config */
            mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "Bobby");
            configAngleMotor();

            /* Drive Motor Config */
            mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "Bobby");
            configDriveMotor();

        lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(new DutyCycleOut(percentOutput, true, true));
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.setControl(new VelocityDutyCycle(velocity, true, Constants.Swerve.feedForward, 0, true));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        mAngleMotor.setControl(new PositionDutyCycle(angle, true, Constants.Swerve.feedForward, 0, true)); 
        lastAngle = angle;
    }

    public void resetToZero() {
        mAngleMotor.setControl(new PositionDutyCycle(moduleConstants.zeroValue, true, Constants.Swerve.feedForward, 0, true));
    }

    public double getTalonAngleMotor(){
        return mAngleMotor.getRotorPosition().getValue();
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.Swerve.angleGearRatio);
        moduleConstants.zeroValue = absolutePosition;
        mAngleMotor.setControl(new PositionDutyCycle(absolutePosition, true, Constants.Swerve.feedForward, 0, true));
    }

    private void configAngleEncoder(){        
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public SwerveModuleState getState(){
        double velocity = mDriveMotor.getVelocity().getValue();
        return new SwerveModuleState(velocity, getCanCoder());
    }

    public SwerveModulePosition getPosition() {
        double distance = mDriveMotor.getRotorPosition().getValue()*Constants.Swerve.wheelCircumference;
        return new SwerveModulePosition(distance, getAngle());
    }
    
    private Rotation2d getAngle(){
        return Rotation2d.fromRotations(mAngleMotor.getRotorPosition().getValue());
    }

 

}