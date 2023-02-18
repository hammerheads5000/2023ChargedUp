package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RegularConstants;
import frc.robot.subsystems.ArmToSetpoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ArmSet extends CommandBase 
{
    //Behold my variables
    private final ArmToSetpoint m_ArmToSetpoint;
    private boolean isFinished;
    public double DesiredX;
    public double DesiredY;
    public ArmSet(ArmToSetpoint subsystem)
    {
        m_ArmToSetpoint = subsystem;
        addRequirements(subsystem);
    }


    @Override
    public void initialize() 
    {
        //puts the desired angle value on smartDashboard to be used
        m_ArmToSetpoint.MoveArm(0,0);
        DesiredX = SmartDashboard.getNumber("XCord", 0);
        DesiredY = SmartDashboard.getNumber("YCord", 0);
    }


    @Override
    public void execute() 
    {
        DesiredX = SmartDashboard.getNumber("XCord", 0);
        DesiredY = SmartDashboard.getNumber("YCord", 0);
        double L1 = RegularConstants.UpperArmLength;
        double L2 = RegularConstants.LowerArmLength;
        double x = DesiredX;
        double y = DesiredY;
        double modulus =  Math.abs((y*y)+(x*x));
        double argument = Math.atan(x/y);
        //law of cosines
        double angleA = Math.acos(((L2*L2)-(L1*L1)-(modulus*modulus))/(2*modulus*L1));
        //law of sines
        double angleB = Math.asin((Math.sin(angleA)*L2)*modulus);
        double LowerDesiredAngle = angleA + argument; 
        double UpperDesiredAngle = angleB;



       //activates arm when it is farther than a degree from desired angle
       // this should be a while loop
        while(m_ArmToSetpoint.AtSetpoint == false)
        {
            m_ArmToSetpoint.MoveArm(UpperDesiredAngle, LowerDesiredAngle);
        }
        if(m_ArmToSetpoint.AtSetpoint == false)
        {
            isFinished = true;
            m_ArmToSetpoint.stop();
            return;
            
        }
        else
        {
            isFinished = false;
            m_ArmToSetpoint.stop();
            return;
        }
       
    }


    @Override
    public void end(boolean interrupted) 
    {
        //stops the arm
   m_ArmToSetpoint.stop();
    }

    
    @Override
    public boolean isFinished() {
      
        return isFinished;
    }   
}

