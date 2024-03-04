package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Autos {

    public SequentialCommandGroup X1() {
        return new SequentialCommandGroup(
            
        );
    }

    public SequentialCommandGroup X2(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoUpper(s_Upper, UpperState.SPEAKER, 0, 0, 0),
            new AutoUpper(s_Upper, UpperState.SHOOT, 0, 0, 0),
            new ParallelCommandGroup(
                new AutoSwerve(s_Swerve, -1.06, 0, 0),
                new AutoUpper(s_Upper, UpperState.GROUND, 0, 0, 0)
            ),
            new AutoUpper(s_Upper, UpperState.TELE, -0.16, -1, 0),
            new AutoUpper(s_Upper, UpperState.SHOOT, 0, 0, 0),
            new ParallelCommandGroup(
                new AutoSwerve(s_Swerve, -4.8, 0.8, 0)
            )
        );
    }

    public SequentialCommandGroup X3() {
        return new SequentialCommandGroup(
            
        );
    }
}
