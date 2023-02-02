package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pivot extends SubsystemBase{

    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
    private MotorControllerGroup pivotGroup;
    private DigitalInput topLimitSwitch;
    private DigitalInput bottomLimitSwitch;

    private DutyCycleEncoder pivotEncoder;

    public Pivot() {
        // Sets CAN ID numbers
        leftPivotMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(61, MotorType.kBrushless);

        
        pivotGroup = new MotorControllerGroup(leftPivotMotor, rightPivotMotor);
        
        pivotEncoder = new DutyCycleEncoder(2);

        topLimitSwitch = new DigitalInput(1);
        bottomLimitSwitch = new DigitalInput(2);

        config();
    }

    public void config() {

        leftPivotMotor.restoreFactoryDefaults();
        leftPivotMotor.burnFlash();
        leftPivotMotor.setIdleMode(IdleMode.kBrake);

        rightPivotMotor.restoreFactoryDefaults();
        rightPivotMotor.burnFlash();
        rightPivotMotor.setIdleMode(IdleMode.kBrake);
        rightPivotMotor.setInverted(true);
        
    }

    public void set(double speed) {
        pivotGroup.set(speed);
    }

    public void pivotUp() {
        pivotGroup.set(0.5);
    }

    public void pivotDown() {
        pivotGroup.set(-0.5);
    }

    public void pivotStop() {
        pivotGroup.set(0.0);
    }

    public double getPivotPosition() {
        return pivotEncoder.getAbsolutePosition(); 
    }

    public boolean getTopSwitch(){
        return topLimitSwitch.get();
    }

    public boolean getBottomSwitch(){
        return bottomLimitSwitch.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot Encoder", getPivotPosition());
    }
    
}
