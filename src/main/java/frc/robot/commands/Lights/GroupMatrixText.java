package frc.robot.commands.Lights;

import com.lumynlabs.domain.led.MatrixTextScrollDirection;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.Lights.LEDSubsystem;

public class GroupMatrixText extends Command {
    private final LEDSubsystem ledSubsystem;
    private final String groupID;
    private final String text;
    private final Color color;
    private final MatrixTextScrollDirection direction;
    private final int delay;
    private final boolean oneShot;

    public GroupMatrixText(LEDSubsystem subsystem, String groupID, String text, Color color, MatrixTextScrollDirection direction, int delay, boolean oneShot) {
        this.ledSubsystem = subsystem;
        this.groupID = groupID;
        this.text = text;
        this.color = color;
        this.direction = direction;
        this.delay = delay;
        this.oneShot = oneShot;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        ledSubsystem.LED_GroupMatrixText(groupID, text, color, direction, delay, oneShot);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}