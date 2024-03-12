// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.UpperSub;
import frc.robot.subsystems.VisionSub;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final Swerve m_Swerve = new Swerve();
  private final UpperSub m_upper = new UpperSub();
  private final VisionSub m_vision = new VisionSub();

  private final XboxController driverController = new XboxController(robotConstants.DriverControllerID);

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, m_vision, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, m_vision, driverController);

  public static String alliance;
  private String command;

  SendableChooser<String> m_Alliance = new SendableChooser<>();
  SendableChooser<String> m_AutoCommand = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(m_Swerve, m_upper, m_vision);

    m_Alliance.setDefaultOption("RED", "RED");
    m_Alliance.addOption("BLUE", "BLUE");

    // left=L mid=M right=R base=B red=r blue=b
    // 1 note
    m_AutoCommand.setDefaultOption("left1n", "left1n");
    m_AutoCommand.addOption("mid1n", "mid1n");
    m_AutoCommand.addOption("right1n", "right1n");

    // 2 note
    m_AutoCommand.addOption("midB2nb", "midB2nb");

    // 3 note
    m_AutoCommand.addOption("leftB3nb", "leftB3nb");

    SmartDashboard.putData("Alliance Team", m_Alliance);
    SmartDashboard.putData("Auto Path", m_AutoCommand);
  }

  @Override
  public void robotPeriodic() {
    alliance = m_Alliance.getSelected();
    command = m_AutoCommand.getSelected();
    // X = m_X.getSelected();
    // Y = m_Y.getSelected();
    // Z = m_Z.getSelected();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    robotConstants.mode = "DISABLED";
  }

  @Override
  public void disabledPeriodic() {
    m_upper.charge(255, 0, 0, (driverController.getLeftY() + driverController.getRightY()));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    robotConstants.mode = "AUTO";
    if(alliance == "RED") {
      switch(command){
        // 1 note
        case "left1n":
          m_autonomousCommand = new Autos().Left_1n(m_Swerve, m_upper);
          break;
        case "mid1n":
          m_autonomousCommand = new Autos().Mid_1n(m_Swerve, m_upper);
          break;
        case "right1n":
          m_autonomousCommand = new Autos().Right_1n(m_Swerve, m_upper);
          break;

        default:
          m_autonomousCommand = null;
          break;
      }
    } else if(alliance == "BLUE") {
      switch(command){
        // 1 note
        case "left1n":
          m_autonomousCommand = new Autos().Left_1n(m_Swerve, m_upper);
          break;
        case "mid1n":
          m_autonomousCommand = new Autos().Mid_1n(m_Swerve, m_upper);
          break;
        case "right1n":
          m_autonomousCommand = new Autos().Right_1n(m_Swerve, m_upper);
          break;

        // 2 note
        case "midB2nb":
          m_autonomousCommand = new Autos().MidX2Base_b2n(m_Swerve, m_upper, m_vision);
          break;

        // 3 note
        case "leftB3nb":
          m_autonomousCommand = new Autos().LeftX1Y1Base_b3n(m_Swerve, m_upper, m_vision);
          break;
        
        default:
          m_autonomousCommand = null;
          break;
      }
    }

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    robotConstants.mode = "TELE";
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_Swerve.setDefaultCommand(teleopSwerve);
    m_upper.setDefaultCommand(teleopUpper);
    m_vision.setDefaultCommand(null);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}