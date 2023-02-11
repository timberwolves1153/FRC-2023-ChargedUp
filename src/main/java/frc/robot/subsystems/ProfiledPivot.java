package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ProfiledPivot  extends ProfiledPIDSubsystem{

    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
   
   // private DigitalInput topLimitSwitch;
   // private DigitalInput bottomLimitSwitch;
    private double pivotVolts;

    private Rotation2d angleOffset;

    private DutyCycleEncoder pivotEncoder;

    public ProfiledPivot() {
        super(new ProfiledPIDController(0, 0, 0, null));

        leftPivotMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(61, MotorType.kBrushless);

        //pivotEncoder = new DutyCycleEncoder(2);

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

        pivotEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);


        
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        // TODO Auto-generated method stub
        
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        return 0;
    }
}
