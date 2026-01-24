package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechanismTelemetry extends SubsystemBase {
    public final Mechanism2d mechanism = new Mechanism2d(1, 1);

    public final MechanismRoot2d pivotRoot = this.mechanism.getRoot("pivotRoot", 0.25, 1);
    public final MechanismLigament2d pivotArm = this.pivotRoot.append(new MechanismLigament2d("", 0, 0, 0, null));

    // todo

    @Override
    public void periodic() {}
}
