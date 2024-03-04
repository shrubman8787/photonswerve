// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.robotConstants;
import frc.robot.commands.AutoAim;
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

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(m_Swerve, driverController);
  private final TeleopUpper teleopUpper = new TeleopUpper(m_upper, driverController);

  private final AutoAim autoAim = new AutoAim(m_Swerve, m_upper, m_vision);

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(m_Swerve, m_upper, m_vision);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = new Autos().X2(m_Swerve, m_upper, m_vision);

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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_Swerve.setDefaultCommand(teleopSwerve);
    m_upper.setDefaultCommand(teleopUpper);
  }

  @Override
  public void teleopPeriodic() {
    if(driverController.getRightBumperPressed()) {
      autoAim.schedule();
    }
  }

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
