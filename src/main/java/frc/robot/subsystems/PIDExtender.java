package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDExtender extends PIDSubsystem {

    private static final int deviceID = 62;
    private CANSparkMax extenderMotor;
    private RelativeEncoder extenderEncoder;
    public DigitalInput extenderLimitSwitch;

    public PIDExtender(){
        super(new PIDController(0.5, 0.001, 0));
        
        extenderMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        extenderMotor.restoreFactoryDefaults();
        extenderMotor.setIdleMode(IdleMode.kBrake);
        extenderMotor.setInverted(true);
        extenderMotor.burnFlash();

        // initialze encoder objects
        extenderEncoder = extenderMotor.getEncoder();
        extenderEncoder.setPosition(0);
        
        // initialze PID controller 
        getController().setTolerance(0);
        getController().setSetpoint(getMeasurement());
        

        extenderLimitSwitch = new DigitalInput(3);

    }

    @Override
    public void useOutput(double output, double setpoint) {
        if (isExtenderInSwitchPressed() && output < 0) {
            extenderPidMove(0);
            
        } else {
            extenderPidMove(output);
        }
    }

    @Override
    public double getMeasurement(){
        return extenderEncoder.getPosition();
    }

    public void extenderPidMove(double volts){
        double clampedVolts = MathUtil.clamp(volts, -6, 12);
        extenderMotor.setVoltage(clampedVolts);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Extender setpoint", getSetpoint());
        SmartDashboard.putNumber("Extender Inches Traveled", encoderTicksToDistance());
        SmartDashboard.putNumber("Extender Output", extenderMotor.getAppliedOutput());
        SmartDashboard.putNumber("Extender Encoder Rotations", getMeasurement());
        SmartDashboard.putBoolean("Extender Limit Switch", isExtenderInSwitchPressed());

        if(isExtenderInSwitchPressed()) {
            resetEncoder();
            setSetpoint(0);
            getController().reset();
        }
    }

    public void holdPosition() {
        setSetpoint(getMeasurement());
        getController().reset();
        enable();
    }

    public void setSetpointInches(double inches) {
        double newSetpoint = distanceToEncoderTicks(inches);
        setSetpoint(newSetpoint);
        getController().reset();
        enable();
    }

    public void extendOut(){
        extenderMotor.setVoltage(7.0);
    }

    public void extendIn(){
        extenderMotor.setVoltage(7.0);
    }

    public void stop(){
        extenderMotor.setVoltage(0.0);
    }

    public boolean isExtenderInSwitchPressed(){
        //vex button defaults to true so we need the opposite
        return !extenderLimitSwitch.get();
    }

    public double distanceToEncoderTicks(double setpoint) {
  
        double encoderTicksPerRev = 25;
        double distancePerRev = Math.PI;
        double conversionFactor = distancePerRev / encoderTicksPerRev;
      
        return setpoint /  conversionFactor;
      }
      
    public double encoderTicksToDistance() {
      
        double encoderTicksPerRev = 25;
        double distancePerRev = Math.PI;
        double conversionFactor = distancePerRev / encoderTicksPerRev;
        double currentTicks = getMeasurement();
      
        return currentTicks * conversionFactor;
      }

    
    public void resetEncoder(){
        extenderEncoder.setPosition(0);
    }

    public boolean isLimitSwitchPressed() {
        return !extenderLimitSwitch.get();

    }


}