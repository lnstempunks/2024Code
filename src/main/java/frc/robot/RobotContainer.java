// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;
    public double startRunTime = 0;
    public double macroTime = 0;
    public int armHold = -1;
    
    // 0 = n/a, 1 = up, 2 = down
    public int armMacroSelector = 0;
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_robotArm = new Arm();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(0);
  public double armStartEncoderValue = m_robotArm.getEncoderValue();
  ArmTarget target;

  private enum ArmTarget {
    AMP(1.5),
    SPEAKER(3.0),
    FLOOR(0.0);

    private final double value;

    ArmTarget(double value) {
        this.value = value;
    }

    public double getValue() {
        return this.value;
    }
  }

  /**armStartEncoderValue
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {


    // Macros

    ParallelCommandGroup shooterMacro = new ParallelCommandGroup(
        new SequentialCommandGroup(
            new WaitCommand(0.2),

            new RunCommand(()->{
                m_shooter.shoot();
            }, m_shooter).withTimeout(2)
        ),
        new SequentialCommandGroup(
            new RunCommand(()->{
                m_intake.intakeBack();
            }, m_intake).withTimeout(0.07),

            new RunCommand(()->{
                m_intake.stopIntake();
            }
            , m_intake).withTimeout(1),

            new RunCommand(()->{
                m_intake.intake();
            }, m_intake).withTimeout(1)
        )
        
    );
    ParallelCommandGroup armMacro = new ParallelCommandGroup(
        new RunCommand(() -> {
            if (m_robotArm.getEncoderValue() < target.getValue()) {
                m_robotArm.armUp();
            } else if (m_robotArm.getEncoderValue() > target.getValue()) {
                m_robotArm.armDown();
            }
        }, m_robotArm).until(() -> {return m_robotArm.getEncoderValue() == target.getValue();})
    );

    // Auto Commands
    NamedCommands.registerCommand("toSpeakerAngle", new RunCommand(()->m_robotArm.armUp(), m_robotArm).withTimeout(0.3));
    NamedCommands.registerCommand("toAmpAngle", new RunCommand(()->m_robotArm.armUp(), m_robotArm).withTimeout(2.1));
    NamedCommands.registerCommand("toFloorSpeaker", new RunCommand(()->m_robotArm.armDown(), m_robotArm).withTimeout(0.3));
    NamedCommands.registerCommand("toFloorAmp", new RunCommand(()->m_robotArm.armDown(), m_robotArm).withTimeout(2.3));
    NamedCommands.registerCommand("toFloorStart", new RunCommand(()->m_robotArm.armDown(), m_robotArm).withTimeout(1.8));
    NamedCommands.registerCommand("toAmpStart", new RunCommand(()->m_robotArm.armUp(), m_robotArm).withTimeout(0.2));
    NamedCommands.registerCommand("shoot", shooterMacro);
    NamedCommands.registerCommand("intake", new RunCommand(()->m_intake.intake(), m_intake).withTimeout(1));
    NamedCommands.registerCommand("armHold", new RunCommand(()->m_robotArm.armHold(), m_robotArm).withTimeout(3.5));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
                if (m_driverController.getR1Button()){
                    m_robotDrive.setX();
                } else{
                m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband));}},
            m_robotDrive));
    m_robotArm.setDefaultCommand(
            new RunCommand(() -> {
                SmartDashboard.putNumber("Arm Encoder", m_robotArm.getEncoderValue());
                SmartDashboard.putNumber("Arm Encoder Start", armStartEncoderValue);
                if(!armMacro.isScheduled()) {
                    if(m_driverController.getTriangleButtonPressed()) {
                        target = ArmTarget.SPEAKER;
                        armMacro.schedule();
                    } else if (m_driverController.getSquareButtonPressed()) {
                        target = ArmTarget.AMP;
                        armMacro.schedule();
                    } else if (m_driverController.getCrossButtonPressed()) {
                        target = ArmTarget.FLOOR;
                        armMacro.schedule();
                    } else if (m_driverController.getCircleButtonPressed()) {
                        m_robotArm.resetEncoder();
                    } else if (m_driverController.getPOV() == 0) {
                        m_robotArm.armUp();
                    } else if (m_driverController.getPOV() == 180) {
                        m_robotArm.armDown();
                    } else {
                        m_robotArm.armHold();
                    }
                }
        }, m_robotArm)
        );


    m_shooter.setDefaultCommand(new RunCommand(() -> {
        if (m_driverController.getR2ButtonPressed()){
            if (shooterMacro.isScheduled() != true){
                shooterMacro.schedule();
            }
        } else if (shooterMacro.isScheduled() != true){
            if (m_driverController.getL1Button()){
                m_shooter.shooterBack();
            } else {
                m_shooter.stopShoot();
            }
        }
    }, m_shooter));

    m_intake.setDefaultCommand(new RunCommand(()-> {
        if (shooterMacro.isScheduled() != true){
            if (m_driverController.getL2Button()) {
                    m_intake.intake();
                } else 
                if (m_driverController.getL1Button()){
                    m_intake.intakeBack();
                } else {
                    m_intake.stopIntake();
                }
        }
    }, m_intake));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
