package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Collector {
    private WPI_TalonSRX collector;
 

    public Collector(){
        
        collector = new WPI_TalonSRX(59);

        config();

    }

    public void config(){
        collector.configFactoryDefault();
        collector.setNeutralMode(NeutralMode.Brake);
    }

    public void set(double speed){
        collector.set(speed);
    }
    public void collectorIntake(){
        collector.set(-0.5);
    }

    public void collectorOuttake(){
        collector.set(-0.5);
    }

    public void collectorStop(){
        collector.set(0.0);
    }
}
