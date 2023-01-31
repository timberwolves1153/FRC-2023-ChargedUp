package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase{
    private CANSparkMax extenderMotor;
    private DigitalInput extenderSwitch;
    private DutyCycleEncoder extenderEncoder;

    public Extender(){
    extenderMotor = new CANSparkMax(70, MotorType.kBrushless);

    extenderSwitch = new DigitalInput(0);
    extenderEncoder = new DutyCycleEncoder(1);

    config();

    }

    public void updateShuffleboard(){
        SmartDashboard.putBoolean("Extender Switch", getExtenderSwitch());
        SmartDashboard.putNumber("Extender Encoder", getExtenderEncoder());

    }

    public void config(){
        extenderMotor.restoreFactoryDefaults();
        extenderMotor.setIdleMode(IdleMode.kBrake);
    }

    public void set(double speed){
        extenderMotor.set(speed);
    }

    public void extendOut(){
        extenderMotor.set(0.3);
    }

    public void extendIn(){
        extenderMotor.set(-0.3);
    }

    public void stop(){
        extenderMotor.set(0.0);
    }

    public boolean getExtenderSwitch(){
        return extenderSwitch.get();
    }

    public double getExtenderEncoder(){
        return extenderEncoder.getAbsolutePosition();
    }
}