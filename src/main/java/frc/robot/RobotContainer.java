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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Configs.CoralSubsystem;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralSubsystem.Setpoint;

public class RobotContainer {

    // The robot's subsystems and commands are defined here...

    private static final String OIConstants = null;
    // Subsystems
        private final AlgaeSubsystem m_algaeSubsystem = new AlgaeSubsystem();
        private final CoralSubsystem m_coralSubSystem = new CoralSubsystem();
    
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
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
        private final Telemetry logger = new Telemetry(MaxSpeed);
    
        private final CommandXboxController m_driverController = new CommandXboxController(0);
        private final CommandXboxController m_operatorController = new CommandXboxController(1);
    
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
            // Autonomous commands
            // Simple drive forward for about 2 meters for 1 seconds
        private final DriveForwardCommand m_DriveForwardCommand = new DriveForwardCommand(drivetrain);
    
    
        public RobotContainer() {
    
    
            configureBindings();
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
    
            
            // TODO Correctly bind these buttons to commands that make the robot do stuff
            // The original REV commands are still here, but some objects, variables, and
            // imnports need to be taken care of
    
            /*
             * If your robot controls don't seem to be going the right direction,
             * rotate so the "front" of the robot faces away from you and is square
             * Hit this start button to reset and recenter
             */
            m_driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    
            // Left Bumper -> Run tube intake
            m_operatorController.leftBumper().whileTrue(m_coralSubSystem.runIntakeCommand());
    
            // Right Bumper -> Run tube intake in reverse
            m_operatorController.rightBumper().whileTrue(m_coralSubSystem.reverseIntakeCommand());
    
            // B Button -> Elevator/Arm to human player position, set ball intake to stow
            // when idle
            
            m_operatorController.b().onTrue(
                            m_coralSubSystem
                                    .setSetpointCommand(Setpoint.kFeederStation)
                                    .alongWith(m_algaeSubsystem.stowCommand()));
            
    
            // Button -> Elevator/Arm to level 2 position
             m_operatorController.a().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel2));
    
            // Button -> Elevator/Arm to level 3 position
             m_operatorController.x().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel3));
    
            // Button -> Elevator/Arm to level 4 position
             m_operatorController.y().onTrue(m_coralSubSystem.setSetpointCommand(Setpoint.kLevel4));
    
            // Trigger -> Run ball intake, set to leave out when idle
        
         m_operatorController
                .rightTrigger(/*OIConstants.kTriggerButtonThreshold*/)
                .whileTrue(m_algaeSubsystem.runIntakeCommand());
        

        // Trigger -> Run ball intake in reverse, set to stow when idle
         m_operatorController
                .leftTrigger(/*OIConstants.kTriggerButtonThreshold*/)
                .whileTrue(m_algaeSubsystem.reverseIntakeCommand());
        
                
        // Brake command if you want it
        // m_driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));


    } // end of configureBindings

    // TODO Autonomously drive
    // 1. Basic - you can add individual auto commands here and swap them out as
    // needed
    // 2. Intermediate - you can add an auto chooser here and select from a list of
    // autos.
    public DriveForwardCommand getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return m_DriveForwardCommand;
        
    }

} // end of RobotContainer

// Point wheels at command
        /*
         * You shouldn't need this so I'm leaving it commented out
         * m_driverController.a().whileTrue(drivetrain.applyRequest(() ->
         * point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(),
         * -m_driverController.getLeftX()))
         * ));
         */