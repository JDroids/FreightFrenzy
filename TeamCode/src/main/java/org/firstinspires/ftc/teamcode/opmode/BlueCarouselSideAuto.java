package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Point;

@Autonomous
public class BlueCarouselSideAuto extends CarouselSideAuto {
    public BlueCarouselSideAuto() {
        super(Alliance.BLUE,
                new Point(10,140),
                new Point(190,140),
                new Point(300,140));
    }
}
