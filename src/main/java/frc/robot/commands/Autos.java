// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  public Command ampToSource(AutoFactory factory, SwerveDrive swerveDrive) {
    AutoLoop loop = new AutoLoop("AmpToSource");

    AutoTrajectory trajectory = factory.trajectory("AmpToSource", loop);

    loop.enabled()
      .onTrue(new InstantCommand(() -> swerveDrive.resetPose(trajectory.getInitialPose().get()))
      .andThen(trajectory.cmd()));

    return loop.cmd();

  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
