// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import javax.sound.midi.Sequencer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.TriggerEvent;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.LightsConstants;
import frc.robot.Constants.ElevatorConstants.ElevateMode;
import frc.robot.Constants.VisionConstants.ScoringMode;
import frc.robot.commands.Arm.ManualArm;
import frc.robot.commands.Arm.ToAngle;
import frc.robot.commands.Climber.Climb;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Drive.DriveToLocation;
import frc.robot.commands.Elevator.ElevateLevel;
import frc.robot.commands.Elevator.ElevateManual;
import frc.robot.commands.Elevator.ElevatorReset;
import frc.robot.commands.Intake.IntakeIn;
//import frc.robot.commands.Intake.IntakeNotifier;
import frc.robot.commands.Wrist.ToWristAngle;
import frc.robot.commands.Wrist.WristMove;
import frc.robot.commands.Intake.IntakeOut;
import frc.robot.commands.Intake.IntakeOut2;
import frc.robot.commands.Intake.IntakeOutL1;
import frc.robot.commands.Intake.IntakeOutVar;
import frc.robot.commands.Intake.Modify;
import frc.robot.commands.Lights.WPIlib.RunPattern;
import frc.robot.commands.Lights.WPIlib.ScrollPattern;
import frc.robot.commands.Lights.WPIlib.SetBreathingPattern;
import frc.robot.commands.Lights.WPIlib.SetSolidColor;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Drive.Vision;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Lights.LEDSubsystem_WPIlib;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Climber.Climber;

public class RobotContainer {
        /* Subsystems */
        final Arm arm = new Arm();
        final Elevator elevator = new Elevator();
        final Intake intake = new Intake();
        final Wrist wrist = new Wrist();
        final LEDSubsystem_WPIlib wpiLights = new LEDSubsystem_WPIlib();
        final Climber climber = new Climber();
        final Vision vision = new Vision();
        public static boolean isModified = false;
        public SendableChooser<Command> m_chooser = new SendableChooser<>();

        public RobotContainer() {
                configureBindings();
                configureNamedCommands();
                configureTestCommands();
                configureAutoCommands();
                // getDashboardCommand();
        }

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */ // TODO: move to a drive command
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        // .withSteerRequestType(SteerRequestType.Position);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        public final CommandXboxController driver = new CommandXboxController(0);
        public final CommandXboxController operator = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        public final Trigger beamBroken = new Trigger(() -> intake.hasCoral());
        public final Trigger modified = new Trigger(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED);
        public final Trigger normal = new Trigger(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL);

        public void idle() {
                getIdleCommands().schedule();
        }

        public void intakeIdle() {
                getIntakeIdleSeq().schedule();
        }

        private void configureBindings() {
                // Nperte that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                drivetrain.registerTelemetry(logger::telemeterize);

                /* DRIVER CONTROLS */
                // drive with joysticks
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed *.5) // Drive
                                                                                                                 // forward
                                                                                                                 // with
                                                                                                                 // negative
                                                                                                                 // Y
                                                                                                                 // (forward)
                                                .withVelocityY(-driver.getLeftX() * MaxSpeed*.5) // Drive left with
                                                                                              // negative X (left)
                                                .withRotationalRate(-driver.getRightX() * MaxAngularRate*.5) // Drive
                                                                                                          // counterclockwise
                                                                                                          // with
                                                                                                          // negative X
                                                                                                          // (left)
                                ));
                // buttons
                // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // driver.b().whileTrue(drivetrain
                // .applyRequest(() -> point.withModuleDirection(
                // new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // CHANGED: changed from back/start+x/y to pov buttons
                // Note that each routine should be run exactly once in a single log.
                // d-pad

                // driver.povUp().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // driver.povDown().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // driver.povLeft().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // driver.povRight().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
                // Bumpers and Triggers
                // reset the field-centric heading on left bumper press
                driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // HP Pickup
                driver.leftBumper().whileTrue(new SequentialCommandGroup(
                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                new ParallelCommandGroup(
                                                new ToWristAngle(() -> Units.degreesToRadians(60), wrist), //used to be 60
                                                new ToAngle(() -> Units.degreesToRadians(1.8), arm), //used to be 1.8
                                                // new IntakeIn(intake),
                                                new ElevateLevel(elevator, ElevateMode.L2))
                                                .finallyDo(this::intakeIdle)));

                driver.rightBumper().whileTrue(new SequentialCommandGroup(
                        new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                        new ParallelCommandGroup(
                                        new ToWristAngle(() -> Units.degreesToRadians(60), wrist), //used to be 60
                                        new ToAngle(() -> Units.degreesToRadians(1.8), arm), //used to be 1.8
                                        // new IntakeIn(intake),
                                        new ElevateLevel(elevator, ElevateMode.L2))));
                driver.rightBumper().onFalse(getIntakeIdleSeq());

                // driver.rightBumper().whileTrue(new SequentialCommandGroup(
                //                 new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                //                 new ParallelCommandGroup(
                //                                 new ToWristAngle(() -> Units.degreesToRadians(34), wrist),
                //                                 new ToAngle(() -> Units.degreesToRadians(-6), arm),
                //                                 new IntakeIn(intake),
                //                                 new ElevateLevel(elevator, ElevateMode.HP))
                //                                 .finallyDo(this::intakeIdle)));
                // 9659 inspired align
                driver.leftTrigger().whileTrue(
                                drivetrain.defer(
                                                () -> DriveToLocation.driveTo(drivetrain.addOffset(true), drivetrain)));
                driver.rightTrigger().whileTrue(
                                drivetrain.defer(() -> DriveToLocation.driveTo(drivetrain.addOffset(false),
                                                drivetrain)));

                /* OPERATOR CONTROLS */
                // joysticks
                arm.setDefaultCommand(new ManualArm(() -> -operator.getLeftY(), arm));
                wrist.setDefaultCommand(new WristMove(() -> -operator.getRightY(), wrist));

                // buttons
                // operator.rightBumper().whileTrue(new IntakeOut(intake));
                operator.leftBumper().whileTrue(new IntakeOut2(intake));
                // modified outtake
                operator.rightBumper().and(operator.x()).whileTrue(
                                new IntakeOutL1(intake));

                operator.rightBumper().and(operator.b())
                                .and(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED).whileTrue(
                                                new IntakeOut(intake));

                operator.rightBumper().and(operator.y())
                                .and(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED)
                                .whileTrue(new IntakeOut(intake));

                operator.rightBumper().and(operator.a()).whileTrue(new IntakeOutVar(intake, () -> -0.2));

                operator.rightBumper().and(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL).and(operator.b())
                                .whileTrue(new IntakeOut2(intake));

                operator.rightBumper().and(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL).and(operator.y())
                                .whileTrue(new IntakeOutVar(intake, () -> 0.7));

                // operator.rightBumper().and(operator.b()).and(operator.rightTrigger()).whileTrue(
                //                 new IntakeOut2(intake));

                // operator.rightBumper().and(operator.a()).and(operator.rightTrigger()).whileTrue(
                //                 new IntakeOut2(intake));

                // operator.x().onTrue(new SetSolidColor(wpiLights, Color.kMagenta));

                // if(drivetrain.getScoringMode() == ScoringMode.NORMAL) {
                // if (drivetrain.decideScoringMode() == ScoringMode.NORMAL) {
                // L4
                operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL)
                                .whileTrue(new SequentialCommandGroup(
                                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                                new ParallelCommandGroup(
                                                                new ToAngle(() -> Units.degreesToRadians(82), arm),
                                                                new ToWristAngle(() -> Units.degreesToRadians(-88),
                                                                                wrist)),
                                                new ElevateLevel(elevator, ElevateMode.L4))
                                                .finallyDo(this::idle));
                // L3
                operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL)
                                .whileTrue(new SequentialCommandGroup(
                                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                                new ParallelCommandGroup(
                                                                new ToWristAngle(() -> Units.degreesToRadians(-80),
                                                                                wrist),
                                                                new ToAngle(() -> Units.degreesToRadians(77), arm)),
                                                new ElevateLevel(elevator, ElevateMode.L3))
                                                .finallyDo(this::idle));
                // L2
                operator.a().whileTrue(new SequentialCommandGroup(
                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(45), arm),
                                                new ToWristAngle(() -> Units.degreesToRadians(-40), wrist)),
                                new ElevateLevel(elevator, ElevateMode.L2))
                                .finallyDo(this::idle));
                // L1
                operator.x().whileTrue(new SequentialCommandGroup(
                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(20), arm),
                                                new ToWristAngle(() -> Units.degreesToRadians(-3), wrist)),
                                new ElevateLevel(elevator, ElevateMode.L1))
                                .finallyDo(this::idle));

                // } else {
                // L4
                operator.y().and(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED)
                                .whileTrue(new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                                new ToAngle(() -> Units.degreesToRadians(77), arm),
                                                                new ToWristAngle(() -> Units.degreesToRadians(3),
                                                                                wrist)),
                                                new ElevateLevel(elevator, ElevateMode.L4))
                                                .finallyDo(this::idle));
                // L3
                operator.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED)
                                .whileTrue(new SequentialCommandGroup(
                                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                                new ParallelCommandGroup(
                                                                new ToWristAngle(() -> Units.degreesToRadians(-58),
                                                                                wrist),
                                                                new ToAngle(() -> Units.degreesToRadians(55), arm)),
                                                new ElevateLevel(elevator, ElevateMode.L3M))
                                                .finallyDo(this::idle));

                // L3 Algae Removal dunk
                driver.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL)
                                .whileTrue(new SequentialCommandGroup(
                                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                                new ParallelCommandGroup(
                                                                new ToWristAngle(() -> Units.degreesToRadians(-53),
                                                                                wrist),
                                                                new ToAngle(() -> Units.degreesToRadians(87), arm)),
                                                new ElevateLevel(elevator, ElevateMode.L3AR)).finallyDo(this::idle));

                // L2 Algae Removal dunk
                driver.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL)
                                .whileTrue(new SequentialCommandGroup(
                                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                                new ParallelCommandGroup(
                                                                new ToWristAngle(() -> Units.degreesToRadians(-76),
                                                                                wrist),
                                                                new ToAngle(() -> Units.degreesToRadians(88.67), arm)),
                                                new ElevateLevel(elevator, ElevateMode.L2AR))
                                                .finallyDo(this::idle));

                // L3 Algae Removal reach
                driver.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED)
                                .whileTrue(new SequentialCommandGroup(
                                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                                new ParallelCommandGroup(
                                                                new ToWristAngle(() -> Units.degreesToRadians(44.5),
                                                                                wrist),
                                                                new ToAngle(() -> Units.degreesToRadians(65), arm)),
                                                new ElevateLevel(elevator, ElevateMode.L3R)).finallyDo(this::idle));
                // l3 algae align
                driver.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED)
                                .whileTrue(drivetrain.defer(
                                                () -> DriveToLocation.driveTo(drivetrain
                                                                .getCenterReefPose(), drivetrain))
                                                .until(() -> drivetrain.getState().Pose == drivetrain
                                                                .getCenterReefPose()));
                // L3 algae align forward
                driver.b().and(() -> drivetrain.decideScoringMode() == ScoringMode.NORMAL).whileTrue(
                                drivetrain.defer(() -> DriveToLocation.driveTo(drivetrain.getCenterReefPose(),
                                                drivetrain)));

                // L2 Algae Removal reach
                driver.a().and(() -> drivetrain.decideScoringMode() == ScoringMode.MODIFIED)
                                .whileTrue(new SequentialCommandGroup(
                                                new ToAngle(() -> Arm.getEncoderPosition().getRadians(), arm),
                                                new ParallelCommandGroup(
                                                                new ToWristAngle(() -> Units.degreesToRadians(-3),
                                                                                wrist),
                                                                new ToAngle(() -> Units.degreesToRadians(25), arm)),
                                                new ElevateLevel(elevator, ElevateMode.L2AR))
                                                .finallyDo(this::idle));

                operator.start().onTrue(new ElevatorReset(elevator));
                beamBroken.onTrue(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("green")));
                beamBroken.onFalse(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("blue")));
                // d-pad
                operator.povDown().whileTrue(new ClimbDown(climber, () -> 1));
                operator.povUp().whileTrue(new Climb(climber, () -> 1));
                operator.povLeft().whileTrue(new IntakeIn(intake)); //should be intake out
                operator.leftTrigger().whileTrue(new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(78), arm), // 75
                                                new ToWristAngle(() -> Units.degreesToRadians(32), wrist)), // 36.5
                                new ElevateLevel(elevator, ElevateMode.L4).withTimeout(0.7),
                                new IntakeOut(intake).withTimeout(0.5)).finallyDo(this::idle));

                // operator.leftTrigger().whileTrue(
                // new SequentialCommandGroup(
                // new ParallelCommandGroup(
                // new ToWristAngle(() -> Units.degreesToRadians(-53),
                // wrist),
                // new ToAngle(() -> Units.degreesToRadians(87), arm)),
                // new ElevateLevel(elevator, ElevateMode.L3AR)).finallyDo(this::idle));

                // operator.leftTrigger().whileTrue(new ParallelCommandGroup(
                // new ToWristAngle(() -> 21.5, wrist),
                // new ToAngle(() -> 2.5, arm),
                // new ElevateLevel(elevator, ElevateMode.HP)));

        }

        public void getDashboardCommand() {
        }

        public Command getIdleCommands() {
                return new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-77), wrist),
                                new ParallelCommandGroup(
                                                new ElevateLevel(elevator, ElevateMode.L1),
                                                new ToAngle(() -> Units.degreesToRadians(60), arm)));
        }

        public Command getIntakeIdleSeq() {
                return new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-76), wrist),
                                new ToAngle(() -> Units.degreesToRadians(20), arm));
        }

        public Command getAutonomousCommand() {
                // return Commands.print("No autonomous command configured");
                return m_chooser.getSelected();
        }

        public void configureTestCommands() {
                // SmartDashboard.putBoolean("is it modified", Intake.modified);
                // SmartDashboard.putData("Elevate", new ElevateLevel(elevator,
                // ElevateMode.TEST));
                // SmartDashboard.putData("Go down", new ElevateLevel(elevator,
                // ElevateMode.DOWN));

                // SmartDashboard.putData("Wrist Up", new ToWristAngle(() -> 70, wrist));
                // SmartDashboard.putData("Wrist Down", new ToWristAngle(() -> -35, wrist));
                // SmartDashboard.putData("Wrist Neuteral", new ToWristAngle(() -> 0, wrist));

        }

        public void configureAutoCommands() {
                m_chooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("autos", m_chooser);
        }

       public void disabledActions() {
                // new SetBreathingPattern(wpiLights, LEDPattern.solid(LightsConstants.GBRColors.get("magenta")), 1);
                // beamBroken.onTrue(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("green")));
                // beamBroken.onFalse(new SetSolidColor(wpiLights, LightsConstants.GRBColors.get("blue")));
                new ScrollPattern(wpiLights, LEDPattern.rainbow(255, 64), 100);
                arm.resetI();
                arm.runState(new TrapezoidProfile.State(Arm.getEncoderPosition().getRadians(), 0));
                wrist.runState(new TrapezoidProfile.State(wrist.getEncoderPosition().getRadians(), 0));
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("L4", new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(82), arm),
                                                new ToWristAngle(() -> Units.degreesToRadians(-88), wrist)),
                                new ElevateLevel(elevator, ElevateMode.L4)).withTimeout(0.7));

                NamedCommands.registerCommand("armToL4", new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-88), wrist),
                                new ToAngle(() -> Units.degreesToRadians(70), arm)));

                NamedCommands.registerCommand("outtake", new IntakeOutVar(intake, () -> 0.6).withTimeout(0.4));

                NamedCommands.registerCommand("intake", new ParallelCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(33), wrist),
                                new ToAngle(() -> Units.degreesToRadians(-7), arm),
                                new IntakeIn(intake),
                                new ElevateLevel(elevator, ElevateMode.HP)).until(() -> intake.hasCoral()));

                NamedCommands.registerCommand("reset", new SequentialCommandGroup(
                                new ToWristAngle(() -> Units.degreesToRadians(-61.5), wrist),
                                new ParallelCommandGroup(
                                                new ToAngle(() -> Units.degreesToRadians(70), arm),
                                                new ElevateLevel(elevator, ElevateMode.L2).withTimeout(0.5))));
                NamedCommands.registerCommand("PathFindLeft", drivetrain.defer(
                                () -> DriveToLocation.driveTo(drivetrain.addOffset(true), drivetrain)));
                NamedCommands.registerCommand("PathFindRight", drivetrain.defer(
                                () -> DriveToLocation.driveTo(drivetrain.addOffset(false), drivetrain)));

        }
}
