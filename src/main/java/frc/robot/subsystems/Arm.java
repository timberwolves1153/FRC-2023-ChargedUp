package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm {
    private CANSparkMax leftArmPivotMotor;
    private CANSparkMax rightArmPivotMotor;
    private CANSparkMax armExtender;

    private MotorControllerGroup armPivot;

    public Arm(){
        leftArmPivotMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightArmPivotMotor = new CANSparkMax(61, MotorType.kBrushless);
        armExtender = new CANSparkMax(62, MotorType.kBrushless);

        armPivot = new MotorControllerGroup(leftArmPivotMotor, rightArmPivotMotor);

        rightArmPivotMotor.setInverted(true);
    }

    public void armPivotFoward(){
        armPivot.set(0.5);
    }

    public void armPivotReverse(){
        armPivot.set(-0.5);
    }

    public void armPivotStop(){
        armPivot.set(0.0);
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
}
