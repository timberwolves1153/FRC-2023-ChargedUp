package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase {

    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    private DutyCycleEncoder pivotEncoder;

    private Rotation2d angleOffset;

    private PIDController pivotController;

   

    public Pivot() {
        // super(new PIDController(0, 0, 0));
        // Sets CAN ID numbers
        leftPivotMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(61, MotorType.kBrushless);
        //leftPivotMotor = new CANSparkMax(6, MotorType.kBrushless);
        //rightPivotMotor = new CANSparkMax(20, MotorType.kBrushless);
       // config();
        
        //pivotGroup = new MotorControllerGroup(leftPivotMotor, rightPivotMotor);
        
        pivotEncoder = new DutyCycleEncoder(0);

       leftPivotMotor.restoreFactoryDefaults();
       rightPivotMotor.restoreFactoryDefaults();

       rightPivotMotor.follow(leftPivotMotor, true);
       leftPivotMotor.setInverted(false);

       leftPivotMotor.setCANTimeout(0);
       rightPivotMotor.setCANTimeout(0);

       leftPivotMotor.setIdleMode(IdleMode.kBrake);
       rightPivotMotor.setIdleMode(IdleMode.kBrake);
       
       leftPivotMotor.burnFlash();
       rightPivotMotor.burnFlash();
       upperLimitSwitch = new DigitalInput(1);
       lowerLimitSwitch = new DigitalInput(2);

       pivotController = new PIDController(0, 0, 0);
       //pivotController.setTolerance(Units.degreesToRadians(5));
       pivotController.setTolerance(.0001);
       pivotController.setSetpoint(getMeasurement());
       
       
    //    SmartDashboard.putNumber("Pivot P", 0);
    //    SmartDashboard.putNumber("Pivot I", 0);
    //    SmartDashboard.putNumber("Pivot D", 0);
    //    SmartDashboard.putNumber("Pivot Setpoint", pivotController.getSetpoint());

       
    
       
      //pivotEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);
      //pivotEncoder.setPositionOffset(0.17);

        // getController().setTolerance(Units.degreesToRadians(5));
    
    }

    public void config() {

        leftPivotMotor.restoreFactoryDefaults();
        leftPivotMotor.burnFlash();
        leftPivotMotor.setIdleMode(IdleMode.kBrake);

        rightPivotMotor.restoreFactoryDefaults();
        rightPivotMotor.burnFlash();
        rightPivotMotor.setIdleMode(IdleMode.kBrake);
        
        
    }
    public void pivotUp() {
        leftPivotMotor.setVoltage(-5.0);
    }

    public void pivotDown() {
        leftPivotMotor.setVoltage(4.0);
    }

    public void pivotStop() {
        leftPivotMotor.setVoltage(0.0);
    }

    public double getPivotPosition() {
        return pivotEncoder.getAbsolutePosition(); 
    }


    public boolean isAtMaxHeight(){
        return upperLimitSwitch.get();
    }

    public boolean isAtMinHeight(){
        return lowerLimitSwitch.get();
    }
    // @Override
    public double getMeasurement() {
        return (pivotEncoder.getAbsolutePosition() + .5) % 1;
    }

    // @Override
    // protected void useOutput(double output, double setpoint) {
    //    double appliedVolts =  MathUtil.clamp(output, -2.5, 2.5);
    //     leftPivotMotor.setVoltage(appliedVolts);
        
    // }



    @Override
    public void periodic(){

        
         SmartDashboard.putNumber("Right Output", rightPivotMotor.getAppliedOutput());
         SmartDashboard.putNumber("Left Output", leftPivotMotor.getAppliedOutput());

         SmartDashboard.putNumber("Pivot Encoder", getMeasurement());
         double rads = getMeasurement() * Math.PI * 2;
         SmartDashboard.putNumber("Pivot Degrees", Units.radiansToDegrees(rads));
         SmartDashboard.putNumber("Pivot Rads", rads);

         SmartDashboard.putBoolean("Upper Limit", isAtMaxHeight());
         SmartDashboard.putBoolean("Lower Limit", isAtMinHeight());

         double p = SmartDashboard.getNumber("Pivot P", 0);
         double i = SmartDashboard.getNumber("Pivot I", 0);
         double d = SmartDashboard.getNumber("Pivot D", 0);
         double setpoint = SmartDashboard.getNumber("Pivot Setpoint", pivotController.getSetpoint());
        
        setpoint = (Math.abs(setpoint) > Math.PI * 2) ? 
            Units.degreesToRadians(setpoint) : setpoint;
         pivotController.setP(p);
         pivotController.setI(i);
         pivotController.setD(d);
         pivotController.setSetpoint(setpoint);
         SmartDashboard.putData("Pivot PID Controller", pivotController);
        double calculatedOutput = pivotController.calculate(getMeasurement(), pivotController.getSetpoint());
         SmartDashboard.putNumber("Calculated Output", calculatedOutput);
         
         double clampedOutput = MathUtil.clamp(calculatedOutput, -7, 6.75);
         SmartDashboard.putNumber("Clamped Output", clampedOutput);
         leftPivotMotor.setVoltage(clampedOutput);
         
    }

    

    

    
}
