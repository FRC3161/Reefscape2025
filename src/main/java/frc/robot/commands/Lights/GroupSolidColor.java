package frc.robot.commands.Lights;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights.LEDSubsystem;
import edu.wpi.first.wpilibj.util.Color;

public class GroupSolidColor extends Command {
    private final LEDSubsystem ledSubsystem;
    private final String groupID;
    private final Color color;

    public GroupSolidColor(LEDSubsystem subsystem, String groupID, Color color) {
        this.ledSubsystem = subsystem;
        this.groupID = groupID;
        this.color = color;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupColor(groupID, color);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}