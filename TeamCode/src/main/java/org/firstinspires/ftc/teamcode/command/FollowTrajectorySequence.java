package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class FollowTrajectorySequence extends CommandBase {
    private SampleMecanumDrive mecanumDrive;
    private TrajectorySequence sequence;

    public FollowTrajectorySequence(SampleMecanumDrive mecanumDrive, TrajectorySequence sequence) {
        this.mecanumDrive = mecanumDrive;
        this.sequence = sequence;
    }

    @Override
    public void initialize() {
        mecanumDrive.followTrajectorySequenceAsync(sequence);
    }

    @Override
    public void execute() {
        mecanumDrive.update();
    }

    @Override
    public void end(boolean interrupted) {
        mecanumDrive.setDrivePower(new Pose2d(0.0, 0.0, 0.0));
    }

    @Override
    public boolean isFinished() {
        return !mecanumDrive.isBusy();
    }
}
