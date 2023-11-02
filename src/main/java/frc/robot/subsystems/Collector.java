package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase{
 
    private WPI_TalonSRX collectorMotor;  //creates an object

    public Collector() {

        collectorMotor = (WPI_TalonSRX) new TalonSRX(51);

        collectorMotor.configContinuousCurrentLimit(40);     //current limit so the motor does not burn out

    }

    public void intake() {   
        collectorMotor.setVoltage(6); //set it between 12 and -12 because thats what the batteries are at

    }

    public void outtake() {
        collectorMotor.setVoltage(-6);
    }
    public void stopCollector() {
        collectorMotor.setVoltage(0); //stops the motors(collector)

    }

}
