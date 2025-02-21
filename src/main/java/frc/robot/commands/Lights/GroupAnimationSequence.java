package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;

public class GroupAnimationSequence extends Command {
    private final LEDSubsystem ledSubsystem;
    private final String groupID;
    private final String sequenceID;
    public GroupAnimationSequence(LEDSubsystem subsystem, String groupID, String sequenceID) {
        this.ledSubsystem = subsystem;
        this.groupID = groupID;
        this.sequenceID = sequenceID;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupAnimationSequence(groupID, sequenceID);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}