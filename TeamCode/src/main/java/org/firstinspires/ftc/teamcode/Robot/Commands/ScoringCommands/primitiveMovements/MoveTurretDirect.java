package org.firstinspires.ftc.teamcode.Robot.Commands.ScoringCommands.primitiveMovements;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;

public class MoveTurretDirect extends Command {
    double delayS = 0.5;

    Turret turret;
    double angle;

    ElapsedTime timer = new ElapsedTime();

    public MoveTurretDirect(Turret turret, double angle) {
        this.turret = turret;
        this.angle = angle;
    }

    @Override
    public void init() {
        turret.setBasedTurretPosition(angle);
        timer.reset();
    }

    @Override
    public void periodic() {

    }

    @Override
    public boolean completed() {
        return timer.seconds() > delayS;
    }

    @Override
    public void shutdown() {

    }
}
