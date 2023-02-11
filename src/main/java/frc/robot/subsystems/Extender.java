package frc.robot.subsystems;

//import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase{
    private static final int deviceID = 62;
    private CANSparkMax extenderMotor;
    private SparkMaxPIDController extenderPidController;
    private RelativeEncoder extenderEncoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public DigitalInput extenderLimitSwitch;
   

    //rotations/distance to ticks
    //private final double distanceToTick = (Math.PI * 0.5)/(countsPerRev);
    //private final double ticksPerInch = (countsPerRev)/(Math.PI * 0.5);



    public Extender(){
        extenderMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        extenderLimitSwitch = new DigitalInput(3);

        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        extenderMotor.restoreFactoryDefaults();

        // initialze PID controller and encoder objects
        extenderPidController = extenderMotor.getPIDController();
        //extenderEncoder = extenderMotor.getEncoder();
        extenderEncoder = extenderMotor.getAlternateEncoder(Type.kQuadrature, 8192);
        extenderPidController.setFeedbackDevice(extenderEncoder);
        extenderEncoder.setPositionConversionFactor(Math.PI * 0.5);

        // PID coefficients
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        // Smart Motion Coefficients
        maxVel = 2000; // rpm
        maxAcc = 1500;

        // set PID coefficients
        extenderPidController.setP(kP);
        extenderPidController.setI(kI);
        extenderPidController.setD(kD);
        extenderPidController.setIZone(kIz);
        extenderPidController.setFF(kFF);
        extenderPidController.setOutputRange(kMinOutput, kMaxOutput);

        /**
         * Smart Motion coefficients are set on a SparkMaxPIDController object
         * 
         * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
         * the pid controller in Smart Motion mode
         * - setSmartMotionMinOutputVelocity() will put a lower bound in
         * RPM of the pid controller in Smart Motion mode
         * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
         * of the pid controller in Smart Motion mode
         * - setSmartMotionAllowedClosedLoopError() will set the max allowed
         * error for the pid controller in Smart Motion mode
         */
        int smartMotionSlot = 0;
        extenderPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        extenderPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        extenderPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        extenderPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);

        // button to toggle between velocity and smart motion modes
    

    }


    public void extendOut(){
        extenderMotor.setVoltage(-6.0);
    }

    public void extendIn(){
        extenderMotor.setVoltage(6.0);
    }

    public void stop(){
        extenderMotor.setVoltage(0.0);
    }

    public boolean getExtenderSwitch(){
        return extenderLimitSwitch.get();
    }

    public double getEncoderPosition(){
        return extenderEncoder.getPosition();
    }

    // public double getEncoderTravelDistance(){
    //     return extenderEncoder.getPosition() * distanceToTick;
    // }

    // public void setDesiredTravelDistance(double desiredDistance){
    //     double desiredTicks = desiredDistance * ticksPerInch;
    //     //Next Step:
    //     //set setpoint to value on smartcontroller
    // }   
    
    public void reset(){
        extenderEncoder.setPosition(0);
    }

    public boolean isLimitSwitchPressed() {
        return !extenderLimitSwitch.get();

    }


    @Override
    public void periodic() {
    
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { extenderPidController.setP(p); kP = p; }
        if((i != kI)) { extenderPidController.setI(i); kI = i; }
        if((d != kD)) { extenderPidController.setD(d); kD = d; }
        if((iz != kIz)) { extenderPidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { extenderPidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          extenderPidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
        if((maxV != maxVel)) { extenderPidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
        if((minV != minVel)) { extenderPidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
        if((maxA != maxAcc)) { extenderPidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
        if((allE != allowedErr)) { extenderPidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
    
        double setPoint, processVariable;
        
          setPoint = SmartDashboard.getNumber("Set Position", 0);
          /**
           * As with other PID modes, Smart Motion is set by calling the
           * setReference method on an existing pid object and setting
           * the control type to kSmartMotion
           */
          extenderPidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
          processVariable = extenderEncoder.getPosition();
        
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("Process Variable", processVariable);
        SmartDashboard.putNumber("Output", extenderMotor.getAppliedOutput());
    
        SmartDashboard.putBoolean("Limit Switch", isLimitSwitchPressed());
    
        if(Math.abs(processVariable - setPoint) < 0.25) {
          extenderMotor.setVoltage(0);
        }
    
        if(isLimitSwitchPressed()) {
          extenderEncoder.setPosition(0);
        }
      }
}
