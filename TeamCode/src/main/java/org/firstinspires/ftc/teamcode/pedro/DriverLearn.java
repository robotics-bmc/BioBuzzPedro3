package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")
public class DriverLearn {


    public class Paths {

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(56, 8, 90);
        private final Pose firstAuto = poseFactory.of(23.591, 45.8384, 180);
        private final Pose point2 = poseFactory.of(19.91, 45.851, 179.805);
        private final Pose point3 = poseFactory.of(16.229, 45.8635, 179.805);
        private final Pose point4 = poseFactory.of(8.8671, 45.8885, -0.195);

        public Path firstAuto() {
            return line(start, firstAuto).linear(start, firstAuto);
        }

        public Path path2() {
            return line(firstAuto, point2).tangent();
        }

        public Path path3() {
            return line(point2, point3).tangent();
        }

        public Path path4() {
            return line(point3, point4).reverseTangent();
        }
    }
}
