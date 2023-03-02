package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RegularConstants;
import frc.robot.subsystems.ArmToSetpoint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
/** An example command that uses an example subsystem. */
public class AutoArmSet extends CommandBase 
{
    //Behold my variables
    private final ArmToSetpoint m_ArmToSetpoint;
    private boolean isFinished;
    public double LastDesiredAngle;
    public double DesiredAngle = 0;
    Timer timer = new Timer();
    private double armMoveSpeed = 0.3;
    DigitalInput limitSwitch = new DigitalInput(5);
    private int angle;

    public AutoArmSet(ArmToSetpoint subsystem, int angle)
    {
        m_ArmToSetpoint = subsystem;
        addRequirements(subsystem);
        this.angle = angle;
    }



    @Override
    public void initialize() 
    {
        //puts the desired angle value on smartDashboard to be used
        m_ArmToSetpoint.MoveArm(SmartDashboard.getNumber("Desired Angle", 0), true);
    }


    @Override
    public void execute() 
    {
        
        
    setAngle();
        
        
    }


    @Override
    public void end(boolean interrupted) 
    {
        //stops the arm
        m_ArmToSetpoint.stop();
    }

    
    @Override
    public boolean isFinished() {
      
        if(timer.get() >= Constants.AutoConstants.autoArmSetTime){
            return true;
          }else{
            return false;
          }
    }   

    public void setAngle()
    {
        Move(angle, true);
        LastDesiredAngle = DesiredAngle;
    }
    
   

    public void Move(double angle, boolean PID)
    {
        m_ArmToSetpoint.MoveArm(angle, true);
        while(Math.abs(m_ArmToSetpoint.AngleDif)>=1)
        {
            if(limitSwitch.get()==true)
            {
                DesiredAngle = 0;
            
            }
            else
            {
                m_ArmToSetpoint.MoveArm(angle, PID);
            }
            
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
}


