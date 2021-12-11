package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Carousel;

public class CarouselForTime extends CommandBase {
    private Carousel carousel;
    private double timeToWait;
    private double power;

    private ElapsedTime timer;

    public CarouselForTime(Carousel carousel, double timeToWait, double power) {
        this.carousel = carousel;
        this.timeToWait = timeToWait;
        this.power = power;

        timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        timer.reset();
        carousel.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        carousel.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > timeToWait;
    }
}
