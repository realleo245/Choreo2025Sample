// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.ProjectFile;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(ExampleSubsystem subsystem, ProjectFile file) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }
  public static Command ampToSource(AutoFactory factory, ProjectFile file, SwerveDrive swerveDrive) {
    Choreo.loadTrajectory("Amp to Source");
    AutoLoop loop = new AutoLoop("AmpToSource");
    AutoTrajectory trajectory = factory.trajectory("AmpToSource", loop);

    loop.enabled()
      .onTrue(new InstantCommand(() -> swerveDrive.resetPose(trajectory.getInitialPose().get()))
      .andThen(trajectory.cmd()));
    

    return loop.cmd();
  }

  public static Command sabotage(AutoFactory factory, ProjectFile file, SwerveDrive swerveDrive) {
    Choreo.loadTrajectory("Sabotage");
    AutoLoop loop = new AutoLoop("Sabotage");
    AutoTrajectory trajectory = factory.trajectory("Sabotage", loop);

    loop.enabled()
      .onTrue(new InstantCommand(() -> swerveDrive.resetPose(trajectory.getInitialPose().get()))
      .andThen(trajectory.cmd()));
    

    return loop.cmd();
  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
