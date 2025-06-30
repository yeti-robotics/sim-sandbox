package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

public class ElevatorSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX primaryElevatorMotor = new TalonFX(ElevatorConfig.primaryElevatorMotorID);
    private final TalonFX secondaryElevatorMotor =
            new TalonFX(ElevatorConfig.secondaryElevatorMotorID);
    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);
    private final NeutralOut neutralOut = new NeutralOut();
    private final MotionMagicTorqueCurrentFOC magicRequest =
            new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    private final StatusSignal<Angle> elevatorPosition = primaryElevatorMotor.getPosition();
    private final StatusSignal<Double> elevatorTargetPosition =
            primaryElevatorMotor.getClosedLoopReference();

    public ElevatorSubsystem() {
        primaryElevatorMotor.getConfigurator().apply(ElevatorConfig.primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(ElevatorConfig.secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfig.primaryElevatorMotorID, true));
        new Trigger(this::getMagSwitch).debounce(2).onTrue(zeroPosition());

        primaryElevatorMotor.setPosition(0);
        secondaryElevatorMotor.setPosition(0);

        primaryElevatorMotor.setControl(neutralOut);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
            PhysicsSim.getInstance().addTalonFX(secondaryElevatorMotor);
        }
    }

    @Override
    public Angle getCurrentPosition() {
        return elevatorPosition.getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(elevatorTargetPosition.getValue());
    }

    private Command zeroPosition() {
        return runOnce(() -> primaryElevatorMotor.setPosition(0));
    }

    public boolean getMagSwitch() {
        return !magSwitch.get();
    }
}
