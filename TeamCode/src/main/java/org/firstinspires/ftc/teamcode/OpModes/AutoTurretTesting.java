package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandFramework.BaseAuto;
import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.CommandFramework.CommandScheduler;
import org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands.TurretToBestCone;
import org.firstinspires.ftc.teamcode.Utils.Team;

@Autonomous
public class AutoTurretTesting extends BaseAuto {
    @Override
    public Command setupAuto(CommandScheduler scheduler) {
        waitForStart();
        return new TurretToBestCone(robot.scoringMechanism.turret, robot.backCamera, false, true);
    }
    @Override
    public Team getTeam() {
        return Team.RED;
    }
}
