package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends TrapezoidProfileSubsystem {
    private CANSparkMax leftArmPivotMotor;
    private CANSparkMax rightArmPivotMotor;
    private CANSparkMax armExtender;
    private DutyCycleEncoder armEncoder;

    private MotorControllerGroup armPivot;
    private SparkMaxPIDController armController;

    private final ArmFeedforward feedforward =
        new ArmFeedforward(ArmConstants.kSVolts, ArmConstants.kGVolts,
            ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

    public Arm() {
        super(
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond, 
                ArmConstants.kMaxAccelerationRadPerSecSquared), 
            ArmConstants.kArmOffsetRads);

            armEncoder = new DutyCycleEncoder(0);
            leftArmPivotMotor = new CANSparkMax(60, MotorType.kBrushless);

            armController = leftArmPivotMotor.getPIDController();
            armController.setP(ArmConstants.kP);
            armController.setI(ArmConstants.kI);
            armController.setD(ArmConstants.kD);
        
        //rightArmPivotMotor = new CANSparkMax(61, MotorType.kBrushless);
        //armExtender = new CANSparkMax(62, MotorType.kBrushless);

        //armEncoder = leftArmPivotMotor.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
        
       // armPivot = new MotorControllerGroup(leftArmPivotMotor, rightArmPivotMotor);

        //rightArmPivotMotor.setInverted(true);
    }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        //calc feedforward from the setpoint
        double calculatedFeedforward = feedforward.calculate(setpoint.position, setpoint.velocity);

        //armController.setpo
    }

    public void armPivotFoward(){
        //armPivot.set(0.3);
        leftArmPivotMotor.set(0.3);
    }

    public void armPivotReverse(){
        //armPivot.set(-0.3);
        leftArmPivotMotor.set(-0.3);
    }

    public void armPivotStop(){
        //armPivot.set(0.0);
        leftArmPivotMotor.set(0.0);
    }

    public void armExtend(){
        armExtender.set(0.5);
    }

    public void armContract(){
        armExtender.set(-0.5);
    }

    public void armExtenderStop(){
        armExtender.set(0.0);
    }
    
    public double getArmPosition() {
        return armEncoder.getAbsolutePosition();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Encoder", getArmPosition());
    }
}
