package frc.robot.subsystems;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Extender extends SubsystemBase{
    private CANSparkMax extenderMotor;
    private DigitalInput extenderSwitch;
    //private SparkMaxAlternateEncoder extenderEncoder;
    private  RelativeEncoder extenderEncoder;
    private final int countsPerRev = 8192;
    private final double distanceToTick = (Math.PI * 0.5)/(countsPerRev);
    private final double ticksPerInch = (countsPerRev)/(Math.PI * 0.5);

    public Extender(){
    extenderMotor = new CANSparkMax(70, MotorType.kBrushless);

    extenderSwitch = new DigitalInput(0);
    //extenderEncoder = new SparkMaxAlternateEncoder(.type.kQuadrature, 8192);
    extenderEncoder = extenderMotor.getAlternateEncoder(Type.kQuadrature, countsPerRev);

    config();

    }

    public void updateShuffleboard(){
        SmartDashboard.putBoolean("Extender Switch", getExtenderSwitch());
        SmartDashboard.putNumber("Extender Encoder", getEncoderPosition());
        SmartDashboard.putNumber("Extender Travel Distance", getEncoderTravelDistance());

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

    public double getEncoderPosition(){
        return extenderEncoder.getPosition();
    }

    public double getEncoderTravelDistance(){
        return extenderEncoder.getPosition() * distanceToTick;
    }

    public void setDesiredTravelDistance(double desiredDistance){
        double desiredTicks = desiredDistance * ticksPerInch;
        //Next Step:
        //set setpoint to value on smartcontroller
    }

    
}