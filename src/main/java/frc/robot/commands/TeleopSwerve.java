package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve m_Swerve;    
    private DoubleSupplier xSup;
    private DoubleSupplier ySup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier m_halfSpeed;
    private BooleanSupplier m_quarterSpeed;
    private BooleanSupplier m_90, m_180, m_270, m_0;

    private double rotationVal, translationVal, strafeVal;
    double m_angle = 0d;
    private PIDController m_thetaController;
    private SendableChooser<Double> m_speedChooser;
  
    /**
     * 
     * @param s_Swerve
     * @param xSup
     * @param ySup
     * @param rotationSup
     * @param robotCentricSup
     * @param halfSpeed
     * @param quarterSpeed
     */
    public TeleopSwerve(Swerve swerve, DoubleSupplier xSup, DoubleSupplier ySup, 
                        DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, 
                        BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed, 
                        BooleanSupplier zero, BooleanSupplier ninety, BooleanSupplier oneEighty, BooleanSupplier twoSeventy){
        m_Swerve = swerve;
        this.ySup = ySup;
        this.xSup = xSup;
        this.rotationSup = rotationSup;
        m_halfSpeed = halfSpeed;
        m_quarterSpeed = quarterSpeed;
        m_0 = zero;
        m_180 = oneEighty;
        m_90 = ninety;
        m_270 = twoSeventy;
        addRequirements(m_Swerve);
       
    }

    @Override 
    public void initialize(){
        m_thetaController = new PIDController(0.05, 0, 0);
        
        m_thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {

        /* Get Values, Deadband*/
        boolean rotateWithButton = m_0.getAsBoolean() || m_90.getAsBoolean() || m_180.getAsBoolean() || m_270.getAsBoolean();
        translationVal = MathUtil.applyDeadband(xSup.getAsDouble(), Constants.stickDeadband);
        strafeVal = MathUtil.applyDeadband(ySup.getAsDouble(), Constants.stickDeadband);
        // SmartDashboard.putBoolean("rotate with button", rotateWithButton);
        
        if(rotateWithButton){
            if(m_0.getAsBoolean()){
                m_thetaController.setSetpoint(0.0);
            }
            else if(m_90.getAsBoolean()){
                m_thetaController.setSetpoint(-90.0);
            }
            else if(m_180.getAsBoolean()){
                m_thetaController.setSetpoint(180.0);
            }
            else if(m_270.getAsBoolean()){
                m_thetaController.setSetpoint(90.0);
            }
            rotationVal = m_thetaController.calculate((MathUtil.inputModulus(m_Swerve.getPose().getRotation().getDegrees(), -180, 180)), m_thetaController.getSetpoint());
            rotationVal = MathUtil.clamp(rotationVal, -Constants.Swerve.maxAngularVelocity * 0.075, Constants.Swerve.maxAngularVelocity * 0.075);
            // SmartDashboard.putNumber("RotationVal", rotationVal);
            // SmartDashboard.putNumber("Theta Controller setpoint", m_thetaController.getSetpoint());
        }
        else if (!rotateWithButton){
            rotationVal = (MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));
        }


        if(m_quarterSpeed.getAsBoolean()){
            translationVal = translationVal*0.25;
            strafeVal =strafeVal*0.25;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.25;
            }
        }
        else if(m_halfSpeed.getAsBoolean()){
            translationVal = translationVal*0.5;
            strafeVal =strafeVal*0.5;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.5;
            }
        }
        else{
            translationVal = translationVal*1.0;
            strafeVal =     strafeVal*1.0;
            if(!rotateWithButton){
                rotationVal = rotationVal *1.0;
            } 
        }
        

        m_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity * 0.9, 
            true,
            false);
    }
}