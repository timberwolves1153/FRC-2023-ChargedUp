package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Pivot extends SubsystemBase {
    private ArmFeedforward pivotFeedforward;
    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    private DutyCycleEncoder pivotEncoder;

    private PIDController pivotController;

   

    public Pivot() {

        pivotFeedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts, 0);
        // Sets CAN ID numbers
        leftPivotMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(61, MotorType.kBrushless);
        
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

       topLimitSwitch = new DigitalInput(1);
       bottomLimitSwitch = new DigitalInput(2);

       pivotController = new PIDController(80, 0, 0);
       //pivotController.setTolerance(Units.degreesToRadians(5));
       pivotController.setTolerance(.0001);
       pivotController.setSetpoint(getMeasurement());
       
       
    //    SmartDashboard.putNumber("Pivot P", 0);
    //    SmartDashboard.putNumber("Pivot I", 0);
    //    SmartDashboard.putNumber("Pivot D", 0);
    //    SmartDashboard.putNumber("Pivot Setpoint", pivotController.getSetpoint());
    
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
        return topLimitSwitch.get();
    }

    public boolean isAtMinHeight(){
        return bottomLimitSwitch.get();
    }
    // @Override
    public double getMeasurement() {
        return (pivotEncoder.getAbsolutePosition() + .5) % 1;
        //return pivotEncoder.getAbsolutePosition();
        

    }

    public double getRadians() {
       return getMeasurement() * 2 * Math.PI;
    }

    
    
    public void teleopMovePivot(double volts) {
        leftPivotMotor.setVoltage(volts);
    }

    public void PIDmovePivot(double volts) {
        //leftPivotMotor.
        double calculatedFeedforward = pivotFeedforward.calculate(getRadians(), 0);
        double calculatedVolts = volts + calculatedFeedforward;//we are trying to find the velocity here);
        double clampedVolts = MathUtil.clamp(calculatedVolts, -7, 5.75);
        if (clampedVolts < 0 && isAtMaxHeight()) {
            leftPivotMotor.setVoltage(0);
        } else if (clampedVolts > 0 && isAtMinHeight()) {
            leftPivotMotor.setVoltage(0);
        } else {
            leftPivotMotor.setVoltage(clampedVolts);
        }
        //leftPivotMotor.setVoltage(clampedVolts);
    }

    @Override
    public void periodic(){

    //     SmartDashboard.putBoolean("Top Limit Switch", isAtMaxHeight());
    //     SmartDashboard.putBoolean("Bottom Limit Switch", isAtMinHeight());

        
    //     SmartDashboard.putNumber("pivot Position", getMeasurement());
    //    // double calculatedOutput = pivotController.calculate(getMeasurement(), pivotController.getSetpoint());
    //    // SmartDashboard.putNumber("output", calculatedOutput);
    //      SmartDashboard.putNumber("Right Output", rightPivotMotor.getAppliedOutput());
    //      SmartDashboard.putNumber("Left Output", leftPivotMotor.getAppliedOutput());

    //      SmartDashboard.putNumber("Pivot Encoder", getMeasurement());
    //      double rads = getMeasurement() * Math.PI * 2;
    //      SmartDashboard.putNumber("Pivot Degrees", Units.radiansToDegrees(rads));
    //      SmartDashboard.putNumber("Pivot Rads", rads);

    //      SmartDashboard.putBoolean("Upper Limit", isAtMaxHeight());
    //      SmartDashboard.putBoolean("Lower Limit", isAtMinHeight());

    //      double p = SmartDashboard.getNumber("Pivot P", 0);
    //      double i = SmartDashboard.getNumber("Pivot I", 0);
    //      double d = SmartDashboard.getNumber("Pivot D", 0);
    //      double setpoint = SmartDashboard.getNumber("Pivot Setpoint", pivotController.getSetpoint());
        
    //     setpoint = (Math.abs(setpoint) > Math.PI * 2) ? 
    //         Units.degreesToRadians(setpoint) : setpoint;
    //      pivotController.setP(p);
    //      pivotController.setI(i);
    //      pivotController.setD(d);
    //      pivotController.setSetpoint(setpoint);
    //      SmartDashboard.putData("Pivot PID Controller", pivotController);
    //     double calculatedOutput = pivotController.calculate(getMeasurement(), pivotController.getSetpoint());
    //      SmartDashboard.putNumber("Calculated Output", calculatedOutput);
         
    //      double clampedOutput = MathUtil.clamp(calculatedOutput, -7, 6.75);
    //      SmartDashboard.putNumber("Clamped Output", clampedOutput);
    //      leftPivotMotor.setVoltage(clampedOutput);
         
    }
}
