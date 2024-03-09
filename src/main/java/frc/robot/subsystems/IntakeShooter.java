package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
    CANSparkMax shooterL = new CANSparkMax(10, MotorType.kBrushless);
    CANSparkMax shooterR = new CANSparkMax(11, MotorType.kBrushless);
    Spark intake = new Spark(0);

    public IntakeShooter(){
        shooterL.setInverted(true);
        shooterR.setInverted(true);
        shooterL.setIdleMode(CANSparkMax.IdleMode.kBrake);
        shooterR.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }
    
    public void shoot() {
        shooterL.set(1);
        shooterR.set(1);
    }
    
    public void stopShoot() {
        shooterL.set(0.0);
        shooterR.set(-0.0);
    }

    public void intake() {
        intake.set(-0.6);
    }

    public void stopIntake() {
        intake.set(0);
    }

    public void unJam(){
        intake.set(0.8);
        shooterL.set(-0.8);
        shooterR.set(-0.8);
    }

}
