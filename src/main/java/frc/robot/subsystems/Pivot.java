package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Pivot extends SubsystemBase{

    private CANSparkMax leftPivotMotor;
    private CANSparkMax rightPivotMotor;
    // private MotorControllerGroup pivotGroup;
   // private DigitalInput topLimitSwitch;
   // private DigitalInput bottomLimitSwitch;

    private DutyCycleEncoder pivotEncoder;

    private double pivotVolts;

    private Rotation2d angleOffset;

    public Pivot() {
        // Sets CAN ID numbers
        leftPivotMotor = new CANSparkMax(60, MotorType.kBrushless);
        rightPivotMotor = new CANSparkMax(61, MotorType.kBrushless);
        //leftPivotMotor = new CANSparkMax(6, MotorType.kBrushless);
        //rightPivotMotor = new CANSparkMax(20, MotorType.kBrushless);
       // config();
        
        //pivotGroup = new MotorControllerGroup(leftPivotMotor, rightPivotMotor);
        
        pivotEncoder = new DutyCycleEncoder(2);

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
      // topLimitSwitch = new DigitalInput(1);
      // bottomLimitSwitch = new DigitalInput(2);

      pivotVolts = 0;
      SmartDashboard.putNumber("Pivot Volts", pivotVolts);
    
      pivotEncoder.setDutyCycleRange(ArmConstants.DUTY_CYCLE_MIN, ArmConstants.DUTY_CYCLE_MAX);
      pivotEncoder.setPositionOffset(0.17);
      angleOffset = new Rotation2d((2 * Math.PI) * 0.17);

        
    }

    public void config() {

        leftPivotMotor.restoreFactoryDefaults();
        leftPivotMotor.burnFlash();
        leftPivotMotor.setIdleMode(IdleMode.kBrake);

        rightPivotMotor.restoreFactoryDefaults();
        rightPivotMotor.burnFlash();
        rightPivotMotor.setIdleMode(IdleMode.kBrake);
        
        
    }

    // public void set(double speed) {
    //     pivotGroup.set(speed);
    // }

    public void pivotUp() {

        //leftPivotMotor.set(-.5);
        //rightPivotMotor.set(.5);
        leftPivotMotor.setVoltage(-5.0);
        rightPivotMotor.setVoltage(5.0);
        //.set(0.5);
    }

    public void pivotDown() {
        //leftPivotMotor.set(.5);
        leftPivotMotor.setVoltage(4.0);
        rightPivotMotor.setVoltage(-4.0);
        //rightPivotMotor.set(-.5);
        //pivotGroup.set(-0.5);
    }

    public void pivotStop() {
        leftPivotMotor.setVoltage(0.0);
        rightPivotMotor.setVoltage(0.0);
    }

    public double getPivotPosition() {
        return pivotEncoder.getAbsolutePosition(); 
    }

    public void testSetVoltage(){
        leftPivotMotor.setVoltage(pivotVolts);
        rightPivotMotor.setVoltage(-pivotVolts);
    }

    // public boolean getTopSwitch(){
    //     return topLimitSwitch.get();
    // }

    // public boolean getBottomSwitch(){
    //     return bottomLimitSwitch.get();
    // }

    public double getOffsetPosition() {
        return pivotEncoder.getAbsolutePosition() - pivotEncoder.getPositionOffset();
    }

    @Override
    public void periodic(){
        
        pivotVolts = SmartDashboard.getNumber("Pivot Volts", 0);

        SmartDashboard.putNumber("Pivot Encoder", getOffsetPosition());
        // SmartDashboard.putBoolean("Right Follows", rightPivotMotor.isFollower());
        // SmartDashboard.putBoolean("Right Inverted", rightPivotMotor.getInverted());
        // SmartDashboard.putBoolean("Left Follows", leftPivotMotor.isFollower());
        // SmartDashboard.putBoolean("Left Inverted", leftPivotMotor.getInverted());
         SmartDashboard.putNumber("Right Output", rightPivotMotor.getAppliedOutput());
         SmartDashboard.putNumber("Left Output", leftPivotMotor.getAppliedOutput());

         double pivotDegrees = Units.radiansToDegrees((2 * Math.PI) * pivotEncoder.getAbsolutePosition());
         SmartDashboard.putNumber("Pivot Degrees", pivotDegrees);
         SmartDashboard.putNumber("Angle Offset", angleOffset.getDegrees());
         
    }
    
}
