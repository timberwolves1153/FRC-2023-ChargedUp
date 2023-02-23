package frc.robot.subsystems;

//import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase{
    private static final int deviceID = 62;
    private CANSparkMax extenderMotor;
    //private final PIDController extenderPidController;
    private RelativeEncoder extenderEncoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public DigitalInput extenderLimitSwitch;
    private double setPoint = 0;

    public Extender(){
        extenderMotor = new CANSparkMax(deviceID, MotorType.kBrushless);
        extenderLimitSwitch = new DigitalInput(3);
        //extenderPidController = new PIDController(0.5, 0.001, 0);
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        extenderMotor.restoreFactoryDefaults();

        // initialze PID controller and encoder objects
        //extenderEncoder = extenderMotor.getEncoder();
        extenderMotor.setInverted(true);
        

        extenderMotor.setIdleMode(IdleMode.kBrake);


        // SmartDashboard.putNumber("Extender P", 0);
        // SmartDashboard.putNumber("Extender I", 0);
        // SmartDashboard.putNumber("Extender D", 0);
        // SmartDashboard.putNumber("Extender Setpoint", extenderPidController.getSetpoint());
        // PID coefficients
       //extenderEncoder.setPosition(0);
        // extenderPidController.setSetpoint(getMeasurement());
        // extenderPidController.setTolerance(0);

    }

    // public PIDController getController() {
    //     return extenderPidController;
    // }

    public double setSetpoint(double setpoint) {
        this.setPoint = setpoint;
        return this.setPoint;
    }

    public double getSetpoint() {
        return this.setPoint;
    }

    public double incrementSetpoint() {        
        this.setPoint++;
        return this.setPoint;
    }

    public double decrementSetpoint() {
        this.setPoint--;
        return this.setPoint;
    }


    public void extendOut(){
        extenderMotor.setVoltage(-7.0);
    }

    public void extendIn(){
        extenderMotor.setVoltage(7.0);
    }

    public void stop(){
        extenderMotor.setVoltage(0.0);
    }

    public boolean getExtenderSwitch(){
        return extenderLimitSwitch.get();
    }

    public double getMeasurement(){
        return extenderEncoder.getPosition();
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

    
    public void reset(){
        extenderEncoder.setPosition(0);
    }

    public boolean isLimitSwitchPressed() {
        return !extenderLimitSwitch.get();

    }
    public void extenderPidMove(double volts){
        double clampedVolts = MathUtil.clamp(volts, -12, 12);

        extenderMotor.setVoltage(clampedVolts);
    }


    @Override
    public void periodic() {
    

        SmartDashboard.putNumber("current extender setpoint", this.setPoint);
        SmartDashboard.putNumber("extender Inches Traveled", encoderTicksToDistance());
    
        SmartDashboard.putNumber("Extender Output", extenderMotor.getAppliedOutput());

        SmartDashboard.putNumber("extender internal Encoder", extenderEncoder.getPosition());

         SmartDashboard.putNumber("Extender Encoder Rotations", getMeasurement());

        //extenderPidMove(extenderPidController.calculate(getMeasurement(), this.setPoint));

        //  double p = SmartDashboard.getNumber("Extender P", 0);
        //  double i = SmartDashboard.getNumber("Extender I", 0);
        //  double d = SmartDashboard.getNumber("Extender D", 0);
        //  double setpoint = SmartDashboard.getNumber("Extender Setpoint", extenderPidController.getSetpoint());
      
        //  extenderPidController.setP(p);
        //  extenderPidController.setI(i);
        //  extenderPidController.setD(d);
        //  extenderPidController.setSetpoint(distanceToEncoderTicks(setpoint));

        //  SmartDashboard.putData("Extender PID Controller", extenderPidController);
        // double calculatedOutput = extenderPidController.calculate(getMeasurement(), extenderPidController.getSetpoint());
        //  SmartDashboard.putNumber("Calculated Output", calculatedOutput);
         
        //  double clampedOutput = MathUtil.clamp(calculatedOutput, -12, 12);
        //  SmartDashboard.putNumber("Clamped Output", clampedOutput);

        //  extenderMotor.setVoltage(clampedOutput);
    }
}
