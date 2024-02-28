package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    CANSparkMax armL = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax armR = new CANSparkMax(12, MotorType.kBrushless);

    public Arm(){
        armL.setInverted(true);
        armR.setInverted(true);
        armL.setIdleMode(CANSparkMax.IdleMode.kBrake);
        armR.setIdleMode(CANSparkMax.IdleMode.kBrake);

    }
    
    public void armUp() {
        armL.set(0.2);
        armR.set(-0.2);
    }

    public void armDown() {
        armL.set(-0.2);
        armR.set(0.2);
    }

    public void armStop() {
        armL.set(0.05);
        armR.set(-0.05);
    }
}
