package org.firstinspires.ftc.teamcode.control.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandGroupBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.opmode.TeleOp.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * A CommandGroups that runs a list of commands in sequence.
 *
 * <p>As a rule, CommandGroups require the union of the requirements of their component commands.
 *
 * @author Jackson
 */
public class DebugSequential extends CommandGroupBase {
    protected Supplier<BiConsumer<String, Object>> telemetrySupplier = Robot.getTelemetry;
    protected BiConsumer<String, Object> telemetryC;
    private final List<Command> m_commands = new ArrayList<>();
    private int m_currentCommandIndex = -1;
    private boolean m_runWhenDisabled = true;

    /**
     * Creates a new SequentialCommandGroup.  The given commands will be run sequentially, with
     * the CommandGroup finishing when the last command finishes.
     *
     * @param commands the commands to include in this group.
     */
    public DebugSequential(Command... commands) {
        telemetryC = telemetrySupplier.get();
        addCommands(commands);
    }


    @Override
    public void addCommands(Command... commands) {
        requireUngrouped(commands);

        if (m_currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a CommandGroup while the group is running");
        }

//        registerGroupedCommands(commands);

        for (Command command : commands) {
            m_commands.add(command);
            m_requirements.addAll(command.getRequirements());
            m_runWhenDisabled &= command.runsWhenDisabled();
        }
    }

    @Override
    public void initialize() {
        m_currentCommandIndex = 0;

        if (!m_commands.isEmpty()) {
            m_commands.get(0).initialize();
        }
    }

    @Override
    public void execute() {
        if (m_commands.isEmpty()) {
            return;
        }

        Command currentCommand = m_commands.get(m_currentCommandIndex);

        telemetryC.accept("Current Running Sequential Member", currentCommand.getName());

        currentCommand.execute();
        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            m_currentCommandIndex++;
            if (m_currentCommandIndex < m_commands.size()) {
                m_commands.get(m_currentCommandIndex).initialize();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && !m_commands.isEmpty()) {
            m_commands.get(m_currentCommandIndex).end(true);
        }
        m_currentCommandIndex = -1;
    }

    @Override
    public boolean isFinished() {
        return m_currentCommandIndex == m_commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return m_runWhenDisabled;
    }

}
