package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDPivot extends PIDSubsystem {
    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
    private DigitalInput upperLimitSwitch;
    private DigitalInput lowerLimitSwitch;
    private DutyCycleEncoder pivotEncoder;
    private final double UNIT_CIRCLE_OFFSET = .561;

    public PIDPivot() {
        super(new PIDController(8.5, 0.05, 0));
        
        //pivotFeedforward = new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts, 0);
        leftPivotMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(61, MotorType.kBrushless);

        pivotEncoder = new DutyCycleEncoder(0);
        upperLimitSwitch = new DigitalInput(1);
        lowerLimitSwitch = new DigitalInput(2);

        leftPivotMotor.restoreFactoryDefaults();
        rightPivotMotor.restoreFactoryDefaults();

        leftPivotMotor.setInverted(false);
        rightPivotMotor.follow(leftPivotMotor, true);

        leftPivotMotor.setIdleMode(IdleMode.kBrake);
        rightPivotMotor.setIdleMode(IdleMode.kBrake);

        leftPivotMotor.burnFlash();
        rightPivotMotor.burnFlash();

        getController().setTolerance(0);
        getController().setSetpoint(getMeasurement());

        enable();
        
    }

    @Override
    public void useOutput(double output, double setpoint) {
        // if (output < 0 && isAtMaxHeight()) {
        //     PIDmovePivot(0);
        // } else if (output > 0 && isAtMinHeight()) {
        //     PIDmovePivot(0);
        // } else {
            PIDmovePivot(output);
        // }
        
    }

    @Override
    public double getMeasurement() {
        //PID controllers need to use radians b/c standard units
        return getRadians();
        //return (pivotEncoder.getAbsolutePosition() + .5) % 1;
    }

        // Only clamps voltage and is called in useOutput()
    public void PIDmovePivot(double volts) {
        double adjustedVolts = volts + 0.35;
            //negative goes up & positive goes down

        double clampedVolts = MathUtil.clamp(adjustedVolts, -5, 5);

        leftPivotMotor.setVoltage(-clampedVolts);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Pivot Setpoint", getSetpoint());
        SmartDashboard.putNumber("Left Pivot Output", leftPivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("Right Pivot Output", rightPivotMotor.getAppliedOutput());
        SmartDashboard.putNumber("PIDPivot Position Radians", getRadians());
        SmartDashboard.putNumber("Pivot Positon Degrees", getDegrees());
        SmartDashboard.putNumber("Pivot Positon Encoder Position", getCorrectedEncounterTicks());
        SmartDashboard.putBoolean("Is at Max height", isAtMaxHeight());
        SmartDashboard.putBoolean("Is at Min Heigh", isAtMinHeight());
        SmartDashboard.putBoolean("PID Pivot Enabled", isEnabled());
    }

    public void pivotUp() {
        leftPivotMotor.setVoltage(-5.0);
            }
        
    public void pivotDown() {
        leftPivotMotor.setVoltage(4.0);
            }
    public void pivotStop() {
        leftPivotMotor.setVoltage(0);
    }

    public double getCorrectedEncounterTicks() {
        //The absolute encoder ticks adjusted to remove the wrap point
        return (pivotEncoder.getAbsolutePosition() + .5) % 1;
    }

    public double getRadians() {
        //gets the radians of the pivot, 
        //adjusted so when the arm is parrallel with the gorund it reads 0 rads
        return (getCorrectedEncounterTicks() - UNIT_CIRCLE_OFFSET) * -2 * Math.PI;
    }

    public double getDegrees() {
        return Math.toDegrees(getRadians());
    }

    public void setSetpointDegrees(double degrees) {
        double newSetpoint = Math.toRadians(degrees);
        setSetpoint(newSetpoint);
        getController().reset();
        enable();
    }

    public void incrementSetpointDegrees() {
        double newSetpoint = Math.toRadians(0.025);
        double oldSetpoint = getController().getSetpoint();
        setSetpoint(newSetpoint + oldSetpoint);
        getController().reset();
        enable();
    }

    public void decrementSetpointDegrees() {
        double newSetpoint = Math.toRadians(0.025);
        double oldSetpoint = getController().getSetpoint();
        setSetpoint(oldSetpoint - newSetpoint);
        getController().reset();
        enable();
    }

    public void teleopMovePivot(double volts) {
        leftPivotMotor.setVoltage(volts);
    }

    

    public void holdPosition() {
        setSetpoint(getMeasurement());
        getController().reset();
        enable();
    }

    public boolean isAtMaxHeight(){
        return upperLimitSwitch.get();
    }

    public boolean isAtMinHeight(){
        return lowerLimitSwitch.get();
    }

    public double ticksToDegrees() {
        return Math.toDegrees((getMeasurement() * 2 * Math.PI));
    }

    public double degreesToTicks(double degrees) {
        return Math.toRadians(degrees)/ (2 * Math.PI);
    }


}