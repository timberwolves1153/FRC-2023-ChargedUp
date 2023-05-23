package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.autos.LimelightAlignLeft;
import frc.robot.autos.LimelightAlignRight;
import frc.robot.subsystems.PIDPivot;
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
  
    private PIDPivot pivot;
    public double pivotDegrees;
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
                        BooleanSupplier quarterSpeed, 
                        BooleanSupplier zero, BooleanSupplier ninety, BooleanSupplier oneEighty, BooleanSupplier twoSeventy, PIDPivot pivot){
        m_Swerve = swerve;
        this.pivot = pivot;
        this.ySup = ySup;
        this.xSup = xSup;
        this.rotationSup = rotationSup;
        //m_halfSpeed = halfSpeed;
        m_quarterSpeed = quarterSpeed;
        m_0 = zero;
        m_180 = oneEighty;
        m_90 = ninety;
        m_270 = twoSeventy;
        
        addRequirements(m_Swerve);
        
        SmartDashboard.putNumber("translation multiplier", 1);
        SmartDashboard.putNumber("strafe multiplier", 1);
        SmartDashboard.putNumber("rotation multiplier", 1);
        
    }

    @Override 
    public void initialize(){
        m_thetaController = new PIDController(0.03, 0, 0.0005);
        
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
            translationVal = translationVal*0.45;
            strafeVal =strafeVal*0.45;
            if(!rotateWithButton){
                rotationVal = rotationVal *0.45;
            }
        }
        // else if(m_halfSpeed.getAsBoolean()){
        //     translationVal = translationVal*0.5;
        //     strafeVal =strafeVal*0.5;
        //     if(!rotateWithButton){
        //         rotationVal = rotationVal *0.5;
        //     }
        //}
        else{
            translationVal = translationVal*1.0;
            strafeVal =     strafeVal*1.0;
            if(!rotateWithButton){
                rotationVal = rotationVal *1.0;
            } 
         }

        

        
        // if(Math.abs(pivot.getDegrees()) < 10) {
        //     double translationMultiplier = 0.65;
        //     double strafeMultiplier = 0.7;
        //     double rotationMultiplier = 1;

        //     translationVal = translationVal * translationMultiplier;
        //     strafeVal = strafeVal * strafeMultiplier;
        //     rotationVal = rotationVal * rotationMultiplier;
        // } else if (pivot.getDegrees() > 10) {
        //     double translationMultiplier = 0.6;
        //     double strafeMultiplier = 0.7;
        //     double rotationMultiplier = 0.9;

        //     translationVal = translationVal * translationMultiplier;
        //     strafeVal = strafeVal * strafeMultiplier;
        //     rotationVal = rotationVal * rotationMultiplier;
        // } else {
        //     double translationMultiplier = 1;
        //     double strafeMultiplier = 1;
        //     double rotationMultiplier = 1;

        //     translationVal = translationVal * translationMultiplier;
        //     strafeVal = strafeVal * strafeMultiplier;
        //     rotationVal = rotationVal * rotationMultiplier;
 
        // }

        m_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity * 0.9, 
            true,
            false);
    }
}