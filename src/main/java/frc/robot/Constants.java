package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

    public static final int ENCODER_COUNTS_PER_METER = 53091;


    public static final double stickDeadband = 0.3;

    public static final class RegularConstants
    {
        /*Motor Arm Values */
        public static final double UpperArmKI = 1/15;
        public static final double LowerArmKI = 1/15;
        public static final double UpperArmRatio = 570 * (42/20) * (42/20);
        public static final double LowerArmRatio = 570 * (40/24); //check this
        public static final double UpperArmMax = .7;
        public static final double LowerArmMax = .6; // test this value
        public static final double UpperArmMin = .1;
        public static final double LowerArmMin = .25; // test this value
        public static final double UpperArmLength = 42;
        public static final double LowerArmLength =45.5;

    }
    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5);
        public static final double wheelBase = Units.inchesToMeters(21+(5/8));
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.14 / 1.0); //8.14:1
        public static final double angleGearRatio = (150.0 / 7.0 / 1.0); //7:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 40;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.45;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.01;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.9;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static double maxSpeed = 4.115; //meters per second
        public static double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 22;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 2;
            public static final double angleOffset = 342.2;
            public static  double zeroValue = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, zeroValue);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final double angleOffset = 308.4;
            public static  double zeroValue = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, zeroValue);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 1;
            public static final double angleOffset = 357.28;
            public static  double zeroValue = .0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, zeroValue);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 24;
            public static final int canCoderID = 3;
            public static final double angleOffset = 144.67;
            public static  double zeroValue = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, zeroValue);
        }
    }

    public static final class AutoConstants {
        public static final PIDConstants anglePID =
            new PIDConstants(0.0, 0.00, 0.00);
        
        public static final PIDConstants drivePID =
            new PIDConstants(0.00, 0.0, 0.0);


        // path following
        public static final double maxVel = 3;
        public static final double maxAcc = 2;

        public static final double toleranceDegrees = 5;
        public static final double tolerancePosition = 0.1;
        public static final double maxDriveSpeed = 2;
        public static final double minDriveSpeed = 0.3;
        public static final double maxAngularVelocityRadians = 2*Math.PI;
        public static final double minAngularVelocityRadians = Math.PI/2;

        // balancing
        public static final double balanceZeroTolerance = 10; // level to stop balancing
        public static final double balanceSpeed = 0.5; // max speed to balance at
        public static final double balanceSensitivity = 0.1; // speed to go / at angle
        public static final double maxBalanceTime = 5;
    }

    public static final class ArmConstants
    {
        public static final double offset = 48;
        public static final double errorChangeTime = 3;
        public static final double speedMultiplierOnDown = 0.9;
        public static final double kP = 0.015;
        public static final double kD = 0.00;   
        public static final double MinAngleWhileDown = 60;
        public static final double MaxAngleWhileUp = 130;
        public static final double armLoweringAngle = 80;
        public static final double presetToleranceDegrees = 5;
        public static final ArmState stow = new ArmState(30, true);
        public static final ArmState upperPlatform = new ArmState(150, false);
        public static final ArmState midPlatform = new ArmState(120, false);
        public static final ArmState ground = new ArmState(50, false);
        public static final ArmState portal = new ArmState(80, true);
        public static final ArmState[] presets = {stow, portal};
    }
}