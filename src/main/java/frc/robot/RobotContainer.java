// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.sim.Mechanisms;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Pick Your Controller and Comment Out the Ones You Don't Use
    private final CommandPS4Controller ps4;
    private final Mechanisms mechanisms;
    private final ElevatorSubsystem elevator;
    private final ArmSubsystem arm;
    private final WristSubsystem wrist;

    public RobotContainer() {
        ps4 = new CommandPS4Controller(Constants.PRIMARY_XBOX_CONTROLLER_PORT);
        mechanisms = new Mechanisms();
        elevator = new ElevatorSubsystem();
        arm = new ArmSubsystem();
        wrist = new WristSubsystem();
        configureBindings();
    }

    public void updateMechanisms() {
        mechanisms.publishComponentPoses(
                elevator.getCurrentPosition(),
                arm.getCurrentPosition(),
                wrist.getCurrentPosition(),
                true);
        mechanisms.publishComponentPoses(
                elevator.getTargetPosition(),
                arm.getTargetPosition(),
                wrist.getTargetPosition(),
                false);
        mechanisms.updateElevatorArmMech(elevator.getCurrentPosition(), arm.getCurrentPosition());
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
