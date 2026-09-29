package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.paths.Path;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/*
 * AutoEx
 *   Phase 1 (PATHS): Pedro follows startToScore() then park().
 *   Phase 2 (BALL):  Pedro lets go of the drive motors and the Limelight
 *                    tracks a yellow ball, holding a 24-inch gap.
 */
@Autonomous (name = "AutoCombo", group = "Auto")
public class AutoEx extends OpMode {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose = poseFactory.of(24, 24, 0);
    private final Pose scorePose = poseFactory.of(48, 48, 90);
    private final Pose parkPose = poseFactory.of(72, 48, 90);

    // ---------------- Ball tracking: hardware ----------------
    private Limelight3A limelight;
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private boolean pathsDone = false;   // set true by the last command in autoRoutine()

    // ---------------- Ball tracking: camera geometry (MEASURE THESE) ----------------
    static final int    BALL_PIPELINE         = 2;     // ← EDIT  Limelight color pipeline for yellow
    static final double CAMERA_HEIGHT_IN      = 8.0;   // ← EDIT  lens center height above floor
    static final double CAMERA_PITCH_DOWN_DEG = 20.0;  // ← EDIT  tilt DOWN from level
    static final double BALL_CENTER_HEIGHT_IN = 2.5 / 2.0;

    // ---------------- Ball tracking: behavior ----------------
    static final double TARGET_DISTANCE_IN    = 24.0;  // 2 feet
    static final double DISTANCE_TOLERANCE_IN = 1.5;   // ← EDIT
    static final double HEADING_TOLERANCE_DEG = 2.0;   // ← EDIT
    static final double KP_DRIVE  = 0.04;              // ← EDIT
    static final double KP_TURN   = 0.02;              // ← EDIT
    static final double MAX_DRIVE = 0.45;              // ← EDIT
    static final double MAX_TURN  = 0.35;              // ← EDIT
    static final double MIN_POWER = 0.06;              // ← EDIT
    static final double SEARCH_TURN_POWER = 0.20;      // ← EDIT
    static final double LOST_TIMEOUT_S    = 0.25;

    private final ElapsedTime lostTimer = new ElapsedTime();
    private double lastTx = 1.0;   // + = ball was last seen right of center

    // Path methods
    private Path startToScore() {
        return line(startPose, scorePose).linear(startPose, scorePose);
    }

    private Path park(){
        return (Path) line(scorePose, parkPose).linear(scorePose, parkPose);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, startToScore()),
                // Add mechanism commands here.
                follow(follower, park()),
                instant(() -> pathsDone = true)   // hand off to ball tracking
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        // ← EDIT  must match the motor names in the Pedro Constants file.
        // These are the same motor objects Pedro configured, so their
        // directions are already set — don't reverse them again here.
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(BALL_PIPELINE);
        limelight.start();   // camera warms up during INIT and the paths
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        if (!pathsDone) {
            // Phase 1: Pedro owns the drivetrain
            follower.update();
            Scheduler.execute();
            telemetry.addData("Phase", "PATHS");
            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
            telemetry.addData("Follower Mode", follower.mode());
        } else {
            // Phase 2: stop calling follower.update() so Pedro stops holding
            // position and the ball tracker can drive the motors.
            telemetry.addData("Phase", "BALL TRACKING");
            trackBall();
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        setDrive(0, 0);
        limelight.stop();
    }

    // =====================================================================
    // Ball tracking — call once per loop(). Non-blocking.
    // =====================================================================
    private void trackBall() {
        LLResult r = limelight.getLatestResult();
        boolean seen = r != null && r.isValid();

        double drive = 0, turn = 0;
        String state;

        if (seen) {
            lostTimer.reset();
            double tx = r.getTx();                          // + = ball right of center
            double dist = distanceInches(r.getTy());
            double distError = dist - TARGET_DISTANCE_IN;   // + = too far away
            lastTx = tx;

            turn = Math.abs(tx) > HEADING_TOLERANCE_DEG ? withMin(KP_TURN * tx, MAX_TURN) : 0;

            if (Math.abs(distError) > DISTANCE_TOLERANCE_IN) {
                // Ease off forward power when off-angle so the ball stays in frame
                double headingScale = Range.clip(1.0 - Math.abs(tx) / 25.0, 0.0, 1.0);
                drive = withMin(KP_DRIVE * distError, MAX_DRIVE) * headingScale;
            }

            state = (drive == 0 && turn == 0) ? "HOLDING 24 in" : "TRACKING";
            telemetry.addData("tx (deg)", "%.1f", tx);
            telemetry.addData("Distance (in)", "%.1f", dist);
        } else if (lostTimer.seconds() < LOST_TIMEOUT_S) {
            state = "BRIEFLY LOST (holding)";
        } else {
            turn = Math.signum(lastTx) * SEARCH_TURN_POWER;
            state = "SEARCHING";
        }

        setDrive(drive, turn);
        telemetry.addData("Ball", state);
        telemetry.addData("Drive / Turn", "%.2f / %.2f", drive, turn);
    }

    /** Floor distance to the ball: (camH - ballH) / tan(pitchDown - ty). */
    private double distanceInches(double tyDeg) {
        double angleDownDeg = Math.max(1.0, CAMERA_PITCH_DOWN_DEG - tyDeg);
        return (CAMERA_HEIGHT_IN - BALL_CENTER_HEIGHT_IN) / Math.tan(Math.toRadians(angleDownDeg));
    }

    /** Clip to ±max and lift small non-zero outputs to MIN_POWER. */
    private double withMin(double value, double max) {
        double v = Range.clip(value, -max, max);
        if (v != 0 && Math.abs(v) < MIN_POWER) v = Math.signum(v) * MIN_POWER;
        return v;
    }

    /** drive = forward/back, turn = rotate (+ = clockwise). */
    private void setDrive(double drive, double turn) {
        double lf = drive + turn, lb = drive + turn;
        double rf = drive - turn, rb = drive - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));
        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }
}