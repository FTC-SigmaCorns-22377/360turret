package org.firstinspires.ftc.teamcode.Robot.Commands.VisionCommands;

import org.firstinspires.ftc.teamcode.CommandFramework.Command;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.ScoringMechanism.Turret;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Vision.BackCamera;
import org.firstinspires.ftc.teamcode.VisionUtils.Cone;

public class TurretToBestCone extends Command {
    private Turret turret;
    private BackCamera backCamera;
    private Cone target = null;
    private boolean allowFar;
    private boolean allowClose;


    public TurretToBestCone(Turret turret, BackCamera backCamera, boolean allowFar, boolean allowClose) {
        super(turret, backCamera);
        this.turret = turret;
        this.backCamera = backCamera;
        this.allowFar = allowFar;
        this.allowClose = allowClose;
    }

    @Override
    public void init() {
        this.target = this.backCamera.getCone(this.allowFar, this.allowClose);
        if (this.target != null) {
            this.turret.setBasedTurretPosition(-1*this.target.position.angle + this.target.position.cameraPosition.getHeading());
        }
    }

    @Override
    public void periodic() {
    }

    @Override
    public boolean completed() { return true; }

    @Override
    public void shutdown() {
        turret.shutdown();
    }


}
