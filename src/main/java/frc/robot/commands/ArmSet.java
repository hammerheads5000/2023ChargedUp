package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.RegularConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
/** An example command that uses an example subsystem. */
public class ArmSet extends CommandBase 
{
    //Behold my variables
    

boolean isFinished;
    @Override
    public void initialize() 
    {
        //puts the desired angle value on smartDashboard to be used
        // SmartDashboard.putNumber("XCord", 0);
        // SmartDashboard.putNumber("YCord", 0);
       // m_UpperArmToSetpoint.MoveArm(SmartDashboard.getNumber("Desired X", 0), true);
       // m_LowerArmToSetpoint.MoveArm(SmartDashboard.getNumber("Desired Y", 0), true);
    }


    @Override
    public void execute() 
    {
     /*   DesiredX = 42;
        DesiredY = 45.5;
        double L1 = RegularConstants.UpperArmLength;
        double L2 = RegularConstants.LowerArmLength;
        double x = DesiredX;
        double y = DesiredY;
        double modulus =  Math.sqrt((y*y)+(x*x));
        SmartDashboard.putNumber("Modulus", modulus);
        double argument = Math.atan(x/y);
        SmartDashboard.putNumber("argument", argument);
        //law of cosines
        double angleA = Math.acos(((L2*L2)-(L1*L1)-(modulus*modulus))/(-2*modulus*L1));
        SmartDashboard.putNumber("angleA", angleA);
        //law of sines
        double angleB = Math.asin((Math.sin(angleA)/L2)*modulus);
        SmartDashboard.putNumber("angleB", angleB);
        double LowerDesiredAngle = (angleA + argument) * 180/Math.PI; 
        double UpperDesiredAngle = angleB *180/Math.PI ;
        SmartDashboard.putNumber("Upper Expected angle", UpperDesiredAngle);
        SmartDashboard.putNumber("Lower Expected Angle", LowerDesiredAngle);
        
        */
    }


    @Override
    public void end(boolean interrupted) 
    {
        //stops the arm
       

    }

    
    @Override
    public boolean isFinished() {
      
        return true;
    }   

   
}

