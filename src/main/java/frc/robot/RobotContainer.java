// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double SlowMoDrive = 0.25; // Increase this from 0 to 1 with 1 being full speed driving
    private double SlowMoTurn = 0.25; // Increase this from 0 to 1 with 1 being full speed turning
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * SlowMoDrive) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * SlowMoDrive) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * SlowMoTurn) // Drive counterclockwise with negative X (left)
            )
        );

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        /*
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        )); */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


    // TODO Correctly bind these buttons to commands that make the robot do stuff
    // The original REV commands are still here, but some objects, variables, and imnports need to be taken care of

    /* 
    // Left Bumper -> Run tube intake
    m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

    // Right Bumper -> Run tube intake in reverse
    m_driverController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());

    // B Button -> Elevator/Arm to human player position, set ball intake to stow
    // when idle
    m_driverController
        .b()
        .onTrue(
            m_coralSubSystem
                .setSetpointCommand(Setpoint.kFeederStation)
                .alongWith(m_algaeSubsystem.stowCommand()));

    // A Button -> Elevator/Arm to level 2 position
    m_driverController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

    // X Button -> Elevator/Arm to level 3 position
    m_driverController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

    // Y Button -> Elevator/Arm to level 4 position
    m_driverController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

    // Right Trigger -> Run ball intake, set to leave out when idle
    m_driverController
        .rightTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.runIntakeCommand());

    // Left Trigger -> Run ball intake in reverse, set to stow when idle
    m_driverController
        .leftTrigger(OIConstants.kTriggerButtonThreshold)
        .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

    */

    } // end of configureBindings

    // TODO Autonomously drive
    // 1. Basic - you can add individual auto commands here and swap them out as needed
    // 2. Intermediate - you can add an auto chooser here and select from a list of autos.
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }




} // end of RobotContainer
