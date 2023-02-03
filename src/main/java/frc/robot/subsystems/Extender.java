package frc.robot.subsystems;

//import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
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

    // SparkMaxPIDController extenderPID;

    // double  kP, 
    //         kI,
    //         kD, 
    //         kIz, 
    //         kFF, 
    //         kMaxOutput, 
    //         kMinOutput,
    //         maxRPM,
    //         maxVel,
    //         minVel,
    //         maxAcc,
    //         allowedError;

    public Extender(){
    extenderMotor = new CANSparkMax(62, MotorType.kBrushless);

    extenderSwitch = new DigitalInput(0);
    //extenderEncoder = new SparkMaxAlternateEncoder(.type.kQuadrature, 8192);
    extenderEncoder = extenderMotor.getAlternateEncoder(Type.kQuadrature, countsPerRev);

    //extenderPID = extenderMotor.getPIDController();

    // //SetPID Constants
    // kP = 5e-5;
    // kI = 1e-6;
    // kD = 0;
    // kIz = 0;
    // kFF = 0.000156;
    // kMaxOutput = 1;
    // kMinOutput = -1;
    // maxRPM = 5700;

    // // smart Motion Coefficients
    // maxVel = 2000; //rpm
    // maxAcc = 1500;

    // extenderPID.setP(kP);
    // extenderPID.setI(kI);
    // extenderPID.setD(kD);
    // extenderPID.setIZone(kIz);
    // extenderPID.setFF(kFF);
    // extenderPID.setOutputRange(kMinOutput, kMaxOutput);

    // int smartMotionSlot = 0;
    // extenderPID.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    // extenderPID.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    // extenderPID.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    // extenderPID.setSmartMotionAllowedClosedLoopError(allowedError, smartMotionSlot);


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

        // extenderPID.setP(kP);
        // extenderPID.setI(kI);
        // extenderPID.setD(kD);
        // extenderPID.setIZone(kIz);
        // extenderPID.setFF(kFF);
        // extenderPID.setOutputRange(kMinOutput, kMaxOutput);
        
    }

    public void set(double speed){
        extenderMotor.set(speed);
    }

    public void extendOut(){
        extenderMotor.set(-0.5);
    }

    public void extendIn(){
        extenderMotor.set(0.5);
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
    
    public void reset(){
        extenderEncoder.setPosition(0);
    }
}