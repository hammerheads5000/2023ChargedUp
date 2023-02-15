package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmToSetpoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class ArmSet extends CommandBase 
{
    //Behold my variables
    private final ArmToSetpoint m_ArmToSetpoint;
    private boolean isFinished;

    public ArmSet(ArmToSetpoint subsystem)
    {
        m_ArmToSetpoint = subsystem;
        addRequirements(subsystem);
    }


    @Override
    public void initialize() 
    {
        //puts the desired angle value on smartDashboard to be used
        m_ArmToSetpoint.MoveArm(SmartDashboard.getNumber("Desired Angle", 0));
    }


    @Override
    public void execute() 
    {
       //activates arm when it is farther than a degree from desired angle
        while(Math.abs(m_ArmToSetpoint.AngleDif)>=1)
        {
            m_ArmToSetpoint.MoveArm(SmartDashboard.getNumber("Desired Angle", 0));
        }
        
        if(Math.abs(m_ArmToSetpoint.AngleDif)>=1)
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

