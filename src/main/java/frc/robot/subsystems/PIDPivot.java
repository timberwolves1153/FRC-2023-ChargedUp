package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class PIDPivot extends PIDSubsystem{
    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;

    private DutyCycleEncoder pivotEncoder;

    

   
    public PIDPivot() {
        super(new PIDController(80, 0, 0));

        //pivotFeedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts, 0);
        
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
       upperLimitSwitch = new DigitalInput(1);
       lowerLimitSwitch = new DigitalInput(2);

       getController().setSetpoint(getMeasurement());
       getController().setTolerance(0);
    }

    @Override
    protected double getMeasurement() {
        return (pivotEncoder.getAbsolutePosition() + .5) % 1;
    }

    @Override
    protected void useOutput(double output, double setpoint) {
        PIDmovePivot(output);
    }

    public void PIDmovePivot(double volts) {
        //negative is up & positive is down
        double clampedVolts = MathUtil.clamp(volts, -7, 6.75);
        leftPivotMotor.setVoltage(clampedVolts);
    }

    public void teleopMovePivot(double volts) {
        leftPivotMotor.setVoltage(volts);
    }

    public void holdPosition() {
        setSetpoint(getMeasurement());
        enable();
    }

    public boolean isAtMaxHeight(){
        return upperLimitSwitch.get();
    }

    public boolean isAtMinHeight(){
        return lowerLimitSwitch.get();
    }

    public double getRadians() {
        return getMeasurement() * 2 * Math.PI;
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public double ticksToDegrees() {
        return Math.toDegrees((getMeasurement() * 2 * Math.PI));
    }

    public double degreesToTicks(double setpoint) {
        return Math.toRadians(setpoint)/ (2 * Math.PI);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Pivot Setpoint", getSetpoint());
        SmartDashboard.putNumber("Left Pivot Output", leftPivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("Right Pivot Output", rightPivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("Current Pivot Positon", getMeasurement());
        SmartDashboard.putNumber("Pivot Position Radians", getRadians());
        SmartDashboard.putNumber("Pivot Positon Degrees", getDegrees());
    }
}