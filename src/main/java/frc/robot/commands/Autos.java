package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperState;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Autos {

    public SequentialCommandGroup ShootCenter(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_CENTERBASE_POS, -1, 1, 0.5)
        );
    }

    public SequentialCommandGroup ShootSide(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.SIDEBASE),
            new AutoUpper(s_Upper, UpperConstants.ELBOW_SIDEBASE_POS, -1, 1, 0.5)
        );
    }

    // these commands are all fucked i think
    // red
    
    public SequentialCommandGroup X1Y1_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.05, 1.18, -0.2),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new ParallelCommandGroup(
                new AutoSwerve(s_Swerve, -1.03, 1.12, -0.07),
                new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
            ),
            new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
            new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
        );
    }

    public SequentialCommandGroup X2Y2Z3_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -0.92, 0, 0),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
            new AutoUpper(s_Upper, -0.16, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -6.68, 1.57, 0),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -4, 1.57, 0),
                new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0, 0, 0),
                new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
            ),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5)
        );
    }

    public SequentialCommandGroup X3Y3_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.07, -1.20, 0.16),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
                ),
                new ParallelCommandGroup(
                    new AutoSwerve(s_Swerve, -1.03, -1.12, 0.07),
                    new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
                    ),
                    new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
                    new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
                    );
    }

    public SequentialCommandGroup X3Leave_r(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new AutoSwerve(s_Swerve, 0, -2, 0),
            new AutoSwerve(s_Swerve, -3, 0, 0)
        );
    }
                
    // blue

    public SequentialCommandGroup X1Y1_b(Swerve s_Swerve, UpperSub s_Upper, VisionSub vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.07, 1.20, -0.16),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new ParallelCommandGroup(
                new AutoSwerve(s_Swerve, -1.03, 1.12, -0.07),
                new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
            ),
            new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
            new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
        );
    }

    public SequentialCommandGroup X2Y2Z3_b(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -0.92, 0, 0),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
            new AutoUpper(s_Upper, -0.16, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -6.68, -1.57, 0),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -4, -1.57, 0),
                new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
            ),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, 0, 0, 0),
                new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
            ),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5)
        );
    }

    public SequentialCommandGroup X3Y3_b(Swerve s_Swerve, UpperSub s_Upper, VisionSub vision) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new ParallelRaceGroup(
                new AutoSwerve(s_Swerve, -1.05, -1.18, 0.2),
                new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
            ),
            new ParallelCommandGroup(
                new AutoSwerve(s_Swerve, -1.03, -1.12, 0.07),
                new AutoUpper(s_Upper, -0.204589, 0, 0, 1.5)
            ),
            new AutoUpper(s_Upper, -0.16, -1, 0, 1.5),
            new AutoUpper(s_Upper, -0.16, -1, 1, 0.5)
        );
    }

    public SequentialCommandGroup X3Leave_b(Swerve s_Swerve, UpperSub s_Upper) {
        return new SequentialCommandGroup(
            new AutoResetEverything(s_Swerve),
            new AutoUpper(s_Upper, UpperState.CENTERBASE),
            new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
            new AutoSwerve(s_Swerve, -1.732, 1, 0),
            new AutoSwerve(s_Swerve, -1.5, -2.598, 0)
        );
    }     
}

    // public SequentialCommandGroup X2Y1Y2Y3_r(Swerve s_Swerve, UpperSub s_Upper, VisionSub s_Vision1) {
    //     return new SequentialCommandGroup(
    //         new AutoUpper(s_Upper, UpperState.SPEAKER),
    //         new AutoUpper(s_Upper, -0.204589, -1, 1, 0.5),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, -1, 1, 0.132),
    //             new AutoUpper(s_Upper, -0.236328, 0.03, 0.3, 5)
    //         ),
    //         new ParallelRaceGroup(
    //             new AutoSwerve(s_Swerve, 0, 0, 0),
    //             new AutoUpper(s_Upper, -0.204589, -1, 0, 5)
    //         )
    //     ); 
    // }