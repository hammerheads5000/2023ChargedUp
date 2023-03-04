package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.RegularConstants;
//import frc.robot.subsystems.ArmToSetpoint;
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
   // private final ArmToSetpoint m_ArmToSetpoint;
    private boolean isFinished;
    public double LastDesiredAngle;
    public double DesiredAngle = 0;
    Timer timer = new Timer();
    private double armMoveSpeed = 0.3;
    DigitalInput limitSwitch = new DigitalInput(5);
    private int angle;

    public AutoArmSet(int angle)
    {
       
        this.angle = angle;
    }



    @Override
    public void initialize() 
    {
        //puts the desired angle value on smartDashboard to be used
      //  m_ArmToSetpoint.MoveArm(SmartDashboard.getNumber("Desired Angle", 0), true);
    }


    @Override
    public void execute() 
    {
        
        
    setAngle();
        
        
    }


    @Override
    public void end(boolean interrupted) 
    {
 
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
      
    }
    
   

}