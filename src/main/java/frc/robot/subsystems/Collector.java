package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase{
    private WPI_TalonSRX collector;
 

    public Collector(){
        
        collector = new WPI_TalonSRX(59);

        config();

    }

    public void config(){
        collector.configFactoryDefault();
        collector.setNeutralMode(NeutralMode.Coast);
    }

    public void collectorIntake(){
        collector.setVoltage(-10.0);
    }

    public void collectorOuttake(){
        collector.setVoltage(10.0);
    }

    public void collectorStop(){
        collector.setVoltage(0.0);
    }
}
