package frc.robot.subsystems.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

public class ArmSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX armKraken = new TalonFX(ArmConfig.ARM_KRAKEN_ID, Constants.RIO_BUS);
    private final NeutralOut neutralOut = new NeutralOut();
    private final MotionMagicTorqueCurrentFOC magicRequest =
            new MotionMagicTorqueCurrentFOC(0).withSlot(0);

    private final StatusSignal<Angle> armPosition = armKraken.getPosition();
    private final StatusSignal<Double> armTargetPosition = armKraken.getClosedLoopReference();
    private CANcoder armEncoder;

    public ArmSubsystem() {
        armKraken.getConfigurator().apply(ArmConfig.talonFXConfiguration);
        armKraken.setControl(neutralOut);
        armEncoder = new CANcoder(ArmConfig.ARM_CANCODER_ID);
        armEncoder.getConfigurator().apply(ArmConfig.cancoderConfiguration);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(armKraken, armEncoder);
        }
    }

    @Override
    public Angle getCurrentPosition() {
        return armPosition.getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(armTargetPosition.getValue());
    }
}
