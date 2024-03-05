// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import com.pathplanner.lib.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public double startRunTime = Timer.getFPGATimestamp();
    public double macroTime = 0;
    public String macroSelector = "Tri";
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_robotArm = new Arm();
  private final IntakeShooter m_intakeShooter = new IntakeShooter();
  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(0);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Configure the button bindings


    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband)),
            m_robotDrive));
    m_robotArm.setDefaultCommand(
            new RunCommand(() -> {
                if (Timer.getFPGATimestamp() > (startRunTime + macroTime)){
                    if (m_driverController.getTriangleButtonPressed()) {
                        startRunTime = Timer.getFPGATimestamp();
                        macroTime = 0.3;
                        macroSelector = "Speaker";
                    } else if (m_driverController.getSquareButtonPressed()) {
                        startRunTime = Timer.getFPGATimestamp();
                        macroTime = 2.1;
                        macroSelector = "Amp";
                    } else if (m_driverController.getCrossButtonPressed()) {
                        startRunTime = Timer.getFPGATimestamp();
                        macroTime = 2;
                        macroSelector = "floorAmp";
                    } else if (m_driverController.getCircleButtonPressed()) {
                        startRunTime = Timer.getFPGATimestamp();
                        macroTime = 0.3;
                        macroSelector = "floorSpeaker";
                    } else if (m_driverController.getPOV() == 0) {
                        m_robotArm.armUp();
                    } else if (m_driverController.getPOV() == 180) {
                        m_robotArm.armDown();
                    } else {
                        m_robotArm.armStop();
                    }
                } else if (macroSelector != "floorAmp") {
                    m_robotArm.armUp();
                } else if (macroSelector != "floorSpeaker") {
                    m_robotArm.armUp();
                } else {
                    m_robotArm.armDown();
                }
        }, m_robotArm)
        );


    m_intakeShooter.setDefaultCommand(new RunCommand(() -> {
        if (m_driverController.getR2Button()) {
            m_intakeShooter.shoot();
        } else {
            m_intakeShooter.stopShoot();
        }
        if (m_driverController.getL2Button()) {
            m_intakeShooter.intake();
        } else 
        if (m_driverController.getL1Button()){
            m_intakeShooter.unJam();
        } else {
            m_intakeShooter.stopIntake();
        }
    }, m_intakeShooter));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory moveOut = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.5, 0.01)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        moveOut,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(moveOut.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.setX());
  }
}
