// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // The robot's subsystems and commands are defined here...

    // Subsystems
    private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
    private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity
    
    // Increase these from 0 to 1 with 1 being full speed driving/turning
    private double SlowMoDrive = 0.75;
    private double SlowMoTurn = 1.5;
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    // private final Telemetry logger = new Telemetry(MaxSpeed);
    
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    // Autonomous commands
    // Simple drive forward for about 2 meters for 1 seconds
    private final DriveForwardCommand m_DriveForwardCommand = new DriveForwardCommand(drivetrain);
    
    
    public RobotContainer() {
        configureBindings();

        // Set the default commands for a algae
        m_algaeSubsystem.setDefaultCommand(m_algaeSubsystem.idleCommand());
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain
                // Drive forward with negative Y (forward)
                        .applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * SlowMoDrive) 
                        // Drive left with negative X (left)
                                .withVelocityY(-m_driverController.getLeftX() * MaxSpeed * SlowMoDrive) 
                                // Drive counterclockwise with negative X (left)
                                .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * SlowMoTurn) 
                        ));
    
        /*
         * If your robot controls don't seem to be going the right direction,
         * rotate so the "front" of the robot faces away from you and is square
         * Hit this start button to reset and recenter
         */
        m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
        // Left Bumper -> Run tube intake
        m_driverController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());

        // Right Bumper -> Run tube intake in reverse
        m_driverController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());

        // B Button -> Elevator/Arm to human player position, set ball intake to stow
        // when idle
        m_operatorController
            .b()
            .onTrue(
                m_coralSubSystem
                    .setSetpointCommand(Setpoint.kFeederStation)
                    .alongWith(m_algaeSubsystem.stowCommand()));

        // A Button -> Elevator/Arm to level 2 position
        m_operatorController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));

        // X Button -> Elevator/Arm to level 3 position
        m_operatorController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));

        // Y Button -> Elevator/Arm to level 4 position
        m_operatorController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));

        // Right Trigger -> Run ball intake, set to leave out when idle
        m_operatorController
            .rightTrigger(Constants.kTriggerButtonThreshold)
            .whileTrue(m_algaeSubsystem.runIntakeCommand());

        // Left Trigger -> Run ball intake in reverse, set to stow when idle
        m_operatorController
            .leftTrigger(Constants.kTriggerButtonThreshold)
            .whileTrue(m_algaeSubsystem.reverseIntakeCommand());

    } // end of configureBindings

    public Command getAutonomousCommand() {
        return m_DriveForwardCommand;
    }

} // end of RobotContainer