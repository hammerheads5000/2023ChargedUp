package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.RegularConstants;
import frc.robot.subsystems.ArmToSetpoint;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
/** An example command that uses an example subsystem. */
public class ArmSet extends CommandBase 
{
    //Behold my variables
    private final ArmToSetpoint m_ArmToSetpoint;
    private boolean isFinished;
    public double LastDesiredAngle;
    public double DesiredAngle = 0;
    private final Joystick driver = new Joystick(0);
    private JoystickButton Increase = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private JoystickButton decrease = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private JoystickButton set = new JoystickButton(driver, XboxController.Button.kA.value);
    private double armMoveSpeed = 0.3;
    DigitalInput limitSwitch = new DigitalInput(5);

    public ArmSet(ArmToSetpoint subsystem)
    {
        m_ArmToSetpoint = subsystem;
        addRequirements(subsystem);
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
        
        if(set.getAsBoolean() == true)
        {
            setAngle();
        }
        else if(Increase.getAsBoolean() == true)
        {
            increaseAngle();
        }
        else if(decrease.getAsBoolean() == true)
        {
            decreaseAngle();
        }
        else
        {
            m_ArmToSetpoint.stop();
        }
        SmartDashboard.putNumber("LastDesiredAngle", LastDesiredAngle);
        SmartDashboard.putNumber("DesiredAngle", DesiredAngle);
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

    public void setAngle()
    {
        DesiredAngle = SmartDashboard.getNumber("Desired Angle", 0);
        Move(DesiredAngle, true);
        LastDesiredAngle = DesiredAngle;
    }
    
    public void increaseAngle()
    {
        m_ArmToSetpoint.moveUp(armMoveSpeed);
    }

    public void decreaseAngle()
    {
        m_ArmToSetpoint.moveDown(armMoveSpeed);
    }

    public void Move(double angle, boolean PID)
    {
        m_ArmToSetpoint.MoveArm(angle, true);
        while(Math.abs(m_ArmToSetpoint.AngleDif)>=1)
        {
            if(limitSwitch.get()==true)
            {
                DesiredAngle = 0;
                m_ArmToSetpoint.Reset();
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



