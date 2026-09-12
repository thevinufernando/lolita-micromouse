#include "straightline_controller.h"
#include "yaw_estimator.h"
#include "turn_controller.h"
#include "wall_follow.h"
#include "tof_sensors.h"
#include "motion_profile.h"

static Controller_t controller;

static uint32_t pid_last_time = 0;
static uint32_t move_start_time = 0;
static float pid_sample_time_s;
static uint16_t settle_counter = 0;

//Debugging
int32_t left_count;
int32_t right_count;
int16_t left_delta_count;
int16_t right_delta_count;
float left_distance;
float right_distance;
float basespeed;
float steering;

//Initialse the controller using the gains from control_config.h
void StraightlineController_Init(void) {

    //Initialise dependencies
    Encoders_Init();
    MotorDriver_Enable();

    //Distance PID: average travelled distance (cm) -> base speed
    controller.distance_pid.Kp        = STRAIGHT_DIST_KP;
    controller.distance_pid.Ki        = STRAIGHT_DIST_KI;
    controller.distance_pid.Kd        = STRAIGHT_DIST_KD;
    controller.distance_pid.tau       = CONTROL_DERIV_TAU_S;
    controller.distance_pid.T         = CONTROL_SAMPLE_TIME_S;
    controller.distance_pid.limMin    = -CONTROL_MAX_SPEED;
    controller.distance_pid.limMax    =  CONTROL_MAX_SPEED;
    controller.distance_pid.limMinInt = -STRAIGHT_DIST_INT_LIMIT;
    controller.distance_pid.limMaxInt =  STRAIGHT_DIST_INT_LIMIT;

    //Heading PID: (left - right) encoder ticks -> steering correction
    controller.straight_pid.Kp        = STRAIGHT_HEADING_KP;
    controller.straight_pid.Ki        = STRAIGHT_HEADING_KI;
    controller.straight_pid.Kd        = STRAIGHT_HEADING_KD;
    controller.straight_pid.tau       = CONTROL_DERIV_TAU_S;
    controller.straight_pid.T         = CONTROL_SAMPLE_TIME_S;
    controller.straight_pid.limMin    = -STRAIGHT_HEADING_LIMIT;
    controller.straight_pid.limMax    =  STRAIGHT_HEADING_LIMIT;
    controller.straight_pid.limMinInt = -STRAIGHT_HEADING_INT_LIMIT;
    controller.straight_pid.limMaxInt =  STRAIGHT_HEADING_INT_LIMIT;

    pid_sample_time_s = controller.distance_pid.T;

    //Initialise PID instances
    PIDController_Init(&controller.straight_pid);
    PIDController_Init(&controller.distance_pid);

    controller.state = STRAIGHTLINE_IDLE;
}

//Helper function to reset PID values
static void resetPID(void) {

    PIDController_Init(&controller.straight_pid);
    PIDController_Init(&controller.distance_pid);

    pid_last_time = 0;
    settle_counter = 0;
}

//Apply the stiction floor without changing the sign of the request
static float applyMinSpeed(float speed) {

    if (CONTROL_MIN_MOVE_SPEED <= 0.0f) return speed;

    if (speed > 0.0f && speed < CONTROL_MIN_MOVE_SPEED) {
        return CONTROL_MIN_MOVE_SPEED;
    }
    if (speed < 0.0f && speed > -CONTROL_MIN_MOVE_SPEED) {
        return -CONTROL_MIN_MOVE_SPEED;
    }

    return speed;
}

//Helper function to update PID controllers
static void updatePID(float target_distance, float direction) {

    //Check if controller is running
    if (controller.state != STRAIGHTLINE_RUNNING) {
        return;
    }

    uint32_t current_time = HAL_GetTick();

    //Safety timeout so a stalled robot cannot spin here forever
    if (current_time - move_start_time >= CONTROL_MOVE_TIMEOUT_MS) {

        controller.state = STRAIGHTLINE_TIMEOUT;
        Motor_Brake();
        return;
    }

    if (current_time - pid_last_time >= (uint32_t)(pid_sample_time_s * 1000.0f)) {

        //Update the last time
        pid_last_time = current_time;

        // Update encoder readings
        Encoders_Update();

        //Signed target: negative when reversing
        float signed_target = target_distance * direction;
        float measured = Encoder_getAverageDistance();

        //Check whether target distance is reached, and stay there a few
        //cycles so we do not declare success while coasting through it
        uint8_t within_tolerance =
            (fabsf(measured - signed_target) < DISTANCE_TOLERANCE_CM);

        if (within_tolerance) {

            settle_counter++;

            if (settle_counter >= CONTROL_SETTLE_CYCLES) {

                //Set state to completed
                controller.state = STRAIGHTLINE_COMPLETED;

                //Stop motors completely
                Motor_Brake();

                return;
            }
        }
        else {
            settle_counter = 0;
        }

        //Debugging
        left_count = Encoder_getLeftCount();
        right_count = Encoder_getRightCount();
        left_delta_count = Encoder_getLeftDeltaCount();
        right_delta_count = Encoder_getRightDeltaCount();
        left_distance = Encoder_getLeftDistance();
        right_distance = Encoder_getRightDistance();

        //Distance PID calculations(update)
        basespeed = PIDController_Update(&controller.distance_pid, signed_target, measured);

        //Straightline PID calculations, measurement in raw encoder ticks
        float straightline_measurement = (float)(Encoder_getLeftCount() - Encoder_getRightCount());

        //Straightline PID update
        steering = PIDController_Update(&controller.straight_pid, 0.0f, straightline_measurement);

        /* ---------------- terminal deadband ----------------
         * Once inside the tolerance band, stop driving and brake instead.
         *
         * Without this, applyMinSpeed() below floors the command to
         * +/-CONTROL_MIN_MOVE_SPEED, so the controller kept shoving the robot
         * at full stiction-breaking torque for the whole CONTROL_SETTLE_CYCLES
         * window it was supposed to be settling in. That impulse is far
         * coarser than STRAIGHT_TOLERANCE_CM, so it routinely knocked the
         * robot straight back out of the band it had just reached -- a limit
         * cycle that ends in CONTROL_MOVE_TIMEOUT_MS rather than a completed
         * move.
         *
         * This is the same defect that was removed from turn_controller.c,
         * where it was the single biggest cause of failed moves. The ratio is
         * worse here: 45 speed units against a 0.7 cm band.
         *
         * Both PIDs are still evaluated above, deliberately, so the
         * integrator and the derivative filter stay in step with the real
         * error instead of seeing a discontinuity when driving resumes. */
        if (within_tolerance) {

            basespeed = 0.0f;
            steering  = 0.0f;
            Motor_Brake();

            return;
        }

        //Overcome gearbox stiction near the target
        float commanded = applyMinSpeed(basespeed);

        // Calculate motor speeds. Steering is differential, so it is added to
        // one wheel and subtracted from the other.
        float left_speed = commanded + steering;
        float right_speed = commanded - steering;

        // Set motor speeds. The signed interface resolves per-wheel direction,
        // so a negative command reverses that wheel instead of wrapping around.
        Motor_runSignedSpeed(left_speed, right_speed);
    }
}

//Helper to reset the state
static void resetStraightlineState(void) {

    //Reset encoders and PID controllers
    Encoders_Reset();
    resetPID();

    move_start_time = HAL_GetTick();

    //Set running state
    controller.state = STRAIGHTLINE_RUNNING;
}

//Shared blocking runner. Returns 1 on success, 0 on timeout.
static uint8_t runDistance(float distance_cm, float direction) {

    //Check whether the controller is in idle state
    if (controller.state != STRAIGHTLINE_IDLE) {

        return 0;
    }

    resetStraightlineState();

    //Run untill the distance is reached
    while (1) {

        //Check whether the controller is finished
        if (controller.state == STRAIGHTLINE_COMPLETED) {

            controller.state = STRAIGHTLINE_IDLE;
            return 1;
        }

        if (controller.state == STRAIGHTLINE_TIMEOUT) {

            controller.state = STRAIGHTLINE_IDLE;
            return 0;
        }

        updatePID(distance_cm, direction);
    }
}

uint8_t runForwardDistance(float distance_cm) {

    return runDistance(distance_cm, 1.0f);
}

uint8_t runBackwardDistance(float distance_cm) {

    return runDistance(distance_cm, -1.0f);
}


/* ==========================================================================
 *            FUSED FORWARD MOVE: gyro heading + one-wall centring
 * ======================================================================== */

static PIDController yaw_pid;

volatile float    sl_yaw_target_deg;
volatile float    sl_yaw_error_deg;
volatile float    sl_steering;
volatile float    sl_basespeed;
volatile uint32_t sl_sat_cycles;
volatile float    sl_ref_cm;
volatile float    sl_align_delta_cm;
volatile uint8_t  sl_align_applied;
volatile uint32_t sl_breakaway_count;
volatile uint8_t  sl_flight_samples;
volatile uint8_t  sl_flight_front;
volatile uint8_t  sl_flight_left;
volatile uint8_t  sl_flight_right;
volatile uint8_t  sl_flight_front_votes;
volatile uint8_t  sl_flight_left_votes;
volatile uint8_t  sl_flight_right_votes;
volatile uint16_t sl_flight_front_mm;
volatile uint16_t sl_flight_left_mm;
volatile uint16_t sl_flight_right_mm;

volatile float    sl_entry_err_mm;
volatile uint8_t  sl_entry_valid;
volatile uint8_t  sl_stall_abort;

/* The round-robin poll refreshes one sensor per control cycle, so a complete
 * rotation is exactly TOF_SENSOR_COUNT cycles -- which is when the wall
 * follower has new data on all three. If the two ever disagree the follower
 * either sees repeated samples or misses some, and neither failure announces
 * itself in the arena. */
_Static_assert(STRAIGHT_TOF_DIVIDER == (uint32_t)TOF_SENSOR_COUNT,
               "STRAIGHT_TOF_DIVIDER must equal TOF_SENSOR_COUNT");

volatile StraightTrace_t tm_sl_trace[SL_TRACE_CAPACITY];
volatile uint32_t        tm_sl_trace_count;


/* Steering priority allocation.
 *
 * base + steer can exceed the motor range, and letting the driver clip each
 * wheel independently silently converts a pure steering command into a net
 * speed change -- the robot stops turning as hard as it was told to, exactly
 * when it is going fastest and needs it most. Reducing the COMMON MODE
 * instead preserves the differential, so the robot gives up speed rather
 * than giving up steering. */
/* ======================= READING WALLS WHILE MOVING ======================
 *
 * The stationary read votes five independent sweeps standing at a cell centre,
 * and costs the 800 ms settle that has to precede it. This does the same
 * counting from samples taken on the way in, over the stretch of the move
 * where the sensors are looking at the cell being entered.
 *
 * SAMPLED ONCE PER ROUND-ROBIN ROTATION, from the caller's ToF rotation, so
 * every sensor contributes one reading per sample and no reading is counted
 * twice. Polling faster would recount held values and make a handful of
 * measurements look like a landslide.
 *
 * AN INVALID READING VOTES "NO WALL". That is not a shortcut -- it is what the
 * stationary read does, and the two must agree or the map would depend on
 * whether the robot happened to be moving when it looked. A side sensor in an
 * open cell legitimately returns nothing at all, which is a real answer. */
static struct {
    uint8_t  samples;
    uint8_t  votes[TOF_SENSOR_COUNT];
    uint16_t valid[TOF_SENSOR_COUNT];
    int32_t  sum_mm[TOF_SENSOR_COUNT];
} s_flight;


static void flightReset(void)
{
    s_flight.samples = 0U;

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {
        s_flight.votes[i]  = 0U;
        s_flight.valid[i]  = 0U;
        s_flight.sum_mm[i] = 0;
    }

    sl_flight_samples     = 0U;
    sl_flight_front       = 0U;
    sl_flight_left        = 0U;
    sl_flight_right       = 0U;
    sl_flight_front_votes = 0U;
    sl_flight_left_votes  = 0U;
    sl_flight_right_votes = 0U;
    sl_flight_front_mm    = TOF_DISTANCE_INVALID;
    sl_flight_left_mm     = TOF_DISTANCE_INVALID;
    sl_flight_right_mm    = TOF_DISTANCE_INVALID;
}


/* One rotation's worth. `remaining_cm` is how much further the robot has to go
 * before it reaches the centre of the cell being read -- positive, and it is
 * what makes the front sensor comparable with a stationary reading. */
static void flightSample(const ToF_Measurement_t m[TOF_SENSOR_COUNT],
                         float remaining_cm)
{
    if (s_flight.samples >= 250U) return;   /* counters are 8-bit */

    s_flight.samples++;

    for (uint8_t i = 0; i < TOF_SENSOR_COUNT; i++) {

        const uint8_t ok = (m[i].valid && m[i].distance_mm != TOF_DISTANCE_INVALID);

        /* THE FRONT SENSOR IS LOOKING PAST THE CELL CENTRE, by however far the
         * robot still has to travel, so its raw reading says nothing directly.
         * What decides a front wall is what it WILL read on arrival -- and
         * subtracting the remaining travel is exact, because the sensor and
         * the wall are both on the axis the robot is moving along.
         *
         * Without this a chained move, which deliberately ends short of the
         * centre, would miss every front wall: one at the far side of the next
         * cell reads about 190 mm from where the segment ends, against a
         * 150 mm threshold meant for a robot standing at the centre. */
        int32_t mm = ok ? (int32_t)m[i].distance_mm : 0;

        if (i == TOF_FRONT && ok) {
            mm -= (int32_t)(remaining_cm * 10.0f);
            if (mm < 0) mm = 0;             /* already inside it */
        }

        const int32_t threshold = (i == TOF_FRONT)
                                ? (int32_t)WALL_FRONT_THRESHOLD_MM
                                : (int32_t)WALL_SIDE_THRESHOLD_MM;

        if (ok) {
            s_flight.valid[i]++;
            s_flight.sum_mm[i] += mm;

            if (mm <= threshold) s_flight.votes[i]++;
        }
    }
}


/* Decide, and publish. Strict majority of the samples taken, which on an even
 * split answers "no wall" -- the safe direction to be wrong in, because a
 * missed wall is seen again from the next cell whereas a phantom one is never
 * cleared. Identical to the rule WallSense_ReadCell() uses. */
static void flightPublish(void)
{
    sl_flight_samples = s_flight.samples;

    const uint8_t n = s_flight.samples;

    sl_flight_front_votes = s_flight.votes[TOF_FRONT];
    sl_flight_left_votes  = s_flight.votes[TOF_LEFT];
    sl_flight_right_votes = s_flight.votes[TOF_RIGHT];

    sl_flight_front = (s_flight.votes[TOF_FRONT] * 2U > n) ? 1U : 0U;
    sl_flight_left  = (s_flight.votes[TOF_LEFT]  * 2U > n) ? 1U : 0U;
    sl_flight_right = (s_flight.votes[TOF_RIGHT] * 2U > n) ? 1U : 0U;

    /* Mean of the VALID samples only, so one dropped reading does not drag the
     * reported distance toward zero. */
    sl_flight_front_mm = s_flight.valid[TOF_FRONT]
        ? (uint16_t)(s_flight.sum_mm[TOF_FRONT] / s_flight.valid[TOF_FRONT])
        : TOF_DISTANCE_INVALID;
    sl_flight_left_mm  = s_flight.valid[TOF_LEFT]
        ? (uint16_t)(s_flight.sum_mm[TOF_LEFT]  / s_flight.valid[TOF_LEFT])
        : TOF_DISTANCE_INVALID;
    sl_flight_right_mm = s_flight.valid[TOF_RIGHT]
        ? (uint16_t)(s_flight.sum_mm[TOF_RIGHT] / s_flight.valid[TOF_RIGHT])
        : TOF_DISTANCE_INVALID;
}


static void allocate(float base, float steer, float *left, float *right)
{
    if (steer >  STRAIGHT_YAW_LIMIT) steer =  STRAIGHT_YAW_LIMIT;
    if (steer < -STRAIGHT_YAW_LIMIT) steer = -STRAIGHT_YAW_LIMIT;

    float headroom = CONTROL_MAX_SPEED - fabsf(steer);

    if (headroom < 0.0f) headroom = 0.0f;

    if (base >  headroom) { base =  headroom; sl_sat_cycles++; }
    if (base < -headroom) { base = -headroom; sl_sat_cycles++; }

    sl_basespeed = base;
    sl_steering  = steer;

    /* !! SIGN !! Positive steer means turn LEFT (anticlockwise, +yaw), and by
     * this project's convention that is right wheel forward, left wheel back.
     * So steer SUBTRACTS from the left wheel. Getting this backwards turns the
     * heading loop into positive feedback: a robot drifting clockwise gets
     * steered further clockwise. Observed as 25-30 degrees of runaway in the
     * first arena run. Cross-check against turn_controller's
     * Motor_runSignedSpeed(-basespeed, +basespeed). */
    *left  = base - steer;
    *right = base + steer;
}


/* Shared implementation of every fused straight move. See StraightMove_t. */
uint8_t runForwardMove(const StraightMove_t *mv)
{
    if (mv == 0 || controller.state != STRAIGHTLINE_IDLE) {
        return 0;
    }

    const float distance_cm     = mv->distance_cm;
    const float front_target_mm = mv->front_target_mm;

    /* A SEGMENT THAT ENDS AT SPEED IS NOT FINISHED WHEN IT RETURNS. It hands
     * the robot over still moving, so it must not brake, must not wait for a
     * settle, and must not zero its own command on the way out. */
    const uint8_t chaining = (mv->exit_speed_cms > 0.0f) && (distance_cm > 0.0f);

    /* Encoders FIRST, then re-base the estimator onto the current yaw. REBASE,
     * not reset: the heading this move inherits from the last turn is exactly
     * what it exists to correct. See yaw_estimator.h.
     *
     * THE RESET IS SKIPPED WHEN THE CALLER TRACKS ITS OWN POSITION, and that
     * is not a saving -- it is what keeps a run on ONE distance axis. See
     * keep_odometry in straightline_controller.h. */
    if (!mv->keep_odometry) {
        Encoders_Reset();

        /* !! THE REBASE BELONGS TO THE RESET AND NOTHING ELSE !!
         *
         * YawEstimator_RebaseEncoders() pins the encoder-yaw channel to
         * wherever the estimate currently is, because zeroing the wheels
         * would otherwise move that measurement out from under it. It adds
         * the current yaw to an origin whose "since reset" term it ASSUMES is
         * now zero.
         *
         * Call it without having reset and it double-counts: the differential
         * travel accumulated so far is still in the sum, so the encoder
         * channel jumps by the whole heading the robot has turned through
         * since the last real reset. The EKF then either rejects every
         * encoder update or, worse, believes some of them. A chained segment
         * must leave the channel exactly as it found it. */
        YawEstimator_RebaseEncoders();
    }

    const float odo0 = Encoder_getAverageDistance();

    if (mv->keep_wall_follow) WallFollow_NewSegment();
    else                      WallFollow_Reset();

    PIDController_Init(&controller.distance_pid);
    PIDController_Init(&yaw_pid);

    yaw_pid.Kp = STRAIGHT_YAW_KP;
    yaw_pid.Ki = STRAIGHT_YAW_KI;
    yaw_pid.Kd = STRAIGHT_YAW_KD;
    yaw_pid.tau = CONTROL_DERIV_TAU_S;
    yaw_pid.T = CONTROL_SAMPLE_TIME_S;
    yaw_pid.limMin = -STRAIGHT_YAW_LIMIT;
    yaw_pid.limMax =  STRAIGHT_YAW_LIMIT;
    yaw_pid.limMinInt = -STRAIGHT_YAW_INT_LIMIT;
    yaw_pid.limMaxInt =  STRAIGHT_YAW_INT_LIMIT;

    /* Same trap as the turn controller: yaw is continuous, so a zeroed
     * derivative history makes the first cycle see a step of the whole
     * accumulated heading. */
    yaw_pid.prevMeasurement = YawEstimator_GetYawDeg();

    settle_counter    = 0;
    sl_sat_cycles     = 0;
    tm_sl_trace_count = 0U;

    MotionProfile_t dist_profile;
    (void)MotionProfile_InitFromTo(&dist_profile, distance_cm,
                                   mv->entry_speed_cms, mv->exit_speed_cms,
                                   STRAIGHT_PROFILE_MAX_CMS,
                                   STRAIGHT_PROFILE_ACCEL_CMS2);

    /* The in-flight wall reading belongs to this segment and nothing else, so
     * it is cleared here whether or not the segment is going to take one. A
     * stale answer served to the next cell is worse than no answer at all. */
    flightReset();

    /* The endpoint, which front-wall alignment may move once.
     *
     * THE PROFILE IS REBUILT, NOT TRANSLATED, and the difference is not
     * cosmetic. The old version added the correction to the profile's output,
     * which moves its ORIGIN by the same amount as its endpoint -- so a
     * correction that shortens the move commands the robot BACKWARDS before it
     * has gone anywhere. Measured: a -2.70 cm correction started the reference
     * at -2.58, the command sat at -45 for 300 ms, and stiction held the robot
     * still for 800.
     *
     * Standing still is not the expensive part. The lateral loop ramps its
     * tilt during those 800 ms, and a heading correction with no forward
     * motion is not a translation, it is a PIVOT -- the robot turned 1.6
     * degrees on the spot and entered the cell already yawed, which is the
     * opposite of what the alignment exists to achieve.
     *
     * So the new endpoint gets a new profile, anchored at the reference
     * position the move has already reached. align_base_cm is where that
     * profile starts and align_t0_s is when, and both are zero until the
     * alignment fires. */
    float   target_cm        = distance_cm;
    float   align_base_cm    = 0.0f;
    float   align_t0_s       = 0.0f;
    uint8_t align_tried      = (front_target_mm > 0.0f) ? 0U : 1U;

    sl_align_delta_cm = 0.0f;
    sl_align_applied  = 0U;
    sl_entry_err_mm   = 0.0f;
    sl_entry_valid    = 0U;
    sl_stall_abort    = 0U;

    /* Breakaway detector state. See the BREAKAWAY PULSE block in
     * control_config.h for why a proportional controller cannot restart this
     * drivetrain on its own. */
    float    prev_measured   = 0.0f;
    uint32_t stall_cycles    = 0;
    uint32_t pulse_until_ms  = 0;
    uint32_t stall_since_ms  = 0;   /* 0 = moving, or trying gently */
    uint8_t  arriving        = 0U;  /* has been inside the band at least once */

    sl_breakaway_count = 0U;

    uint32_t start_ms = HAL_GetTick();
    /* Both seeded from the move's own start, never from 0: a tick counter that
     * has been running for minutes would otherwise make the first measured
     * period enormous. */
    uint32_t last_pid = start_ms;
    uint32_t last_tof = start_ms;
    uint32_t tof_div  = 0;
    float    tilt_deg = 0.0f;

    controller.state = STRAIGHTLINE_RUNNING;

    while (1) {

        YawEstimator_Predict();

        uint32_t now = HAL_GetTick();

        if (now - start_ms >= CONTROL_MOVE_TIMEOUT_MS) {
            Motor_Brake();
            /* The robot is not where this segment was aiming, so whatever the
             * sensors saw on the way belongs to no cell anyone can name. */
            flightReset();
            controller.state = STRAIGHTLINE_IDLE;
            return 0;
        }

        if (now - last_pid < (uint32_t)(CONTROL_SAMPLE_TIME_S * 1000.0f)) {
            continue;
        }

        /* THE PERIOD IS MEASURED, NOT ASSUMED, and the difference is not
         * small. A ToF sweep blocks this loop for 138 ms, so the real cadence
         * is 10, 10, 10, 138 and repeat -- 81% of a move's wall-clock time is
         * spent inside one of those reads with the motors holding a stale
         * command.
         *
         * Telling a PID that a 138 ms step took 10 ms multiplies its
         * derivative by 13.8. The trace shows exactly that: across one sweep
         * the heading error moved 5.66 deg, the D term read it as
         * 0.20 * 5.66 / 0.010 = 113 units, P added 42, and the steering
         * clamped at its limit. With the true period it would have been 8
         * units and nothing would have saturated. Every steering slam in the
         * late half of that run is this arithmetic, not a collision.
         *
         * The blocking read is the real defect and wants fixing in the ToF
         * driver. Until then the loop must at least be honest about how long
         * it has been away. */
        float dt_s = (float)(now - last_pid) * 0.001f;

        /* Bounded so a debugger halt or a lost I2C transaction cannot hand the
         * PIDs a period long enough to wind the integrator in one step. */
        if (dt_s > CONTROL_SAMPLE_TIME_S * 40.0f) dt_s = CONTROL_SAMPLE_TIME_S * 40.0f;
        if (dt_s < CONTROL_SAMPLE_TIME_S)         dt_s = CONTROL_SAMPLE_TIME_S;

        last_pid = now;

        controller.distance_pid.T = dt_s;
        yaw_pid.T                 = dt_s;

        YawEstimator_Correct();
        YawEstimator_PublishTelemetry();

        float measured   = Encoder_getAverageDistance() - odo0;
        float elapsed_s  = (float)(now - start_ms) * 0.001f;

        /* HOW FAST THE ROBOT IS ACTUALLY GOING, measured once and used by
         * three different decisions below -- whether the move is finished,
         * whether to fire a breakaway pulse, and whether to give up. They were
         * separately re-deriving it or, worse, not asking at all. */
        const float travelled = fabsf(measured - prev_measured);
        const float speed_cms = travelled / dt_s;

        prev_measured = measured;

        /* Profile time, which is move time until the alignment rebuilds the
         * profile and restarts its clock. */
        float prof_t_s   = elapsed_s - align_t0_s;

        float ref_pos    = align_base_cm
                           + MotionProfile_Position(&dist_profile, prof_t_s);
        float ref_vel    = MotionProfile_Velocity(&dist_profile, prof_t_s);

        sl_ref_cm = ref_pos;

        /* FINISHED MEANS CLOSE ENOUGH *AND* STOPPED, and the second half was
         * missing.
         *
         * The old test was position only, so a robot crossing into the
         * tolerance band at full speed declared the move complete and then
         * carried on for however far it took to stop. Measured: a median of
         * 11.2 cm/s against a profile asking for 10, exiting a 1.5 cm band,
         * and front-wall stops landing 22 to 25 mm past target.
         *
         * That overshoot would be a rounding error if the robot only ever
         * drove straight. It is not, because a 90 degree turn converts
         * longitudinal error into LATERAL error almost one for one -- a cell
         * that stopped 22 mm short was followed by a move that began 17 mm off
         * centre, against every other entry error in that run being inside
         * 7 mm. The completion tolerance was setting the floor on how well
         * placed the robot could possibly be after any turn, and no amount of
         * lateral tuning gets underneath it. */
        /* ARRIVAL IS LATCHED. Once the robot has been close enough once, it
         * commits to stopping and the band is not consulted again.
         *
         * Without the latch the speed condition would make things worse rather
         * than better: a robot that coasts through the band coasts back OUT of
         * it, the completion test un-arms, the command returns, and it hunts
         * -- eventually backwards, on a breakaway pulse, which is the one
         * direction this chassis has no lateral sensing for. Latching turns
         * "close enough" into a decision made once. */
        /* A CHAINED SEGMENT IS FINISHED WHEN IT GETS THERE, full stop.
         *
         * None of the arrival machinery below applies: it is all about coming
         * to rest at a point, and this segment is meant to be travelling when
         * it reaches one. There is no band either -- the handover point is
         * taken exactly, and whatever lag the robot has at that moment is
         * measured and absorbed by the next segment rather than waited out.
         *
         * NO BRAKE ON THE WAY OUT. The motors keep their last command while
         * the caller decides what happens next, which is the whole mechanism:
         * see MAZE_CONTINUOUS_CELLS. */
        if (chaining) {

            if (measured >= target_cm) {
                flightPublish();
                controller.state = STRAIGHTLINE_IDLE;
                return 1;
            }
        }
        else {

            if (!arriving && fabsf(measured - target_cm) < DISTANCE_TOLERANCE_CM) {
                arriving = 1U;
            }

            if (arriving && speed_cms < STRAIGHT_SETTLE_SPEED_CMS) {

                settle_counter++;
                if (settle_counter >= CONTROL_SETTLE_CYCLES) {
                    Motor_Brake();
                    flightPublish();
                    controller.state = STRAIGHTLINE_IDLE;
                    return 1;
                }
            }
            else {
                settle_counter = 0;
            }
        }

        /* ONE SENSOR EVERY CYCLE, rather than three every fourth.
         *
         * The same I2C work, unbunched. Three at once cost 35 ms and the loop
         * was stopped for all of it -- measured at 10, 10, 10, 35 repeating,
         * with 53% of a move spent not running. One at a time keeps every
         * cycle near twelve, and each sensor is still refreshed every
         * STRAIGHT_TOF_DIVIDER cycles, inside TOF_INTER_MEASUREMENT_MS, so
         * nothing is actually sampled less often.
         *
         * Latest, not newest-only: on the cycles where a sensor has nothing
         * new the held reading is served, because handing the wall follower an
         * invalid measurement would make it drop and re-acquire its reference
         * several times a second. */
        ToF_Measurement_t m[TOF_SENSOR_COUNT];

        (void)ToF_PollOneLatest(m, TOF_MAX_SAMPLE_AGE_MS);

        /* The wall follower runs once per complete rotation, when every sensor
         * has been refreshed exactly once since it last looked. Running it on
         * every cycle would feed it two thirds repeated data and make its slew
         * limit and integral -- both rates -- act on samples that had not
         * changed. */
        if (++tof_div >= STRAIGHT_TOF_DIVIDER) {
            tof_div = 0;

            /* Its own interval, measured. The slew limit and the integral are
             * rates, so they need the gap they are actually integrated over
             * rather than the nominal one. */
            float tof_dt_s = (float)(now - last_tof) * 0.001f;

            if (tof_dt_s > WALL_FOLLOW_UPDATE_S * 10.0f)
                tof_dt_s = WALL_FOLLOW_UPDATE_S * 10.0f;
            if (tof_dt_s < WALL_FOLLOW_UPDATE_S)
                tof_dt_s = WALL_FOLLOW_UPDATE_S;

            /* `measured` is how far into the move the robot is, which is what
             * tells the follower whether its side sensors are still looking at
             * the cell being left or already at the one being entered. This
             * module supplies the distance and nothing else -- which cells
             * those are, and what the map knows about them, is the navigator's
             * business. */
            tilt_deg = WallFollow_Update(m, tof_dt_s, measured);

            /* ONE ROTATION, ONE VOTE. Every sensor has been refreshed exactly
             * once since the last time round, which is what makes these
             * samples independent -- the same argument that makes the
             * stationary read vote fresh sweeps rather than re-reads. */
            if (mv->wall_window_cm >= 0.0f && measured >= mv->wall_window_cm) {
                float remaining = mv->wall_centre_cm - measured;

                if (remaining < 0.0f) remaining = 0.0f;

                flightSample(m, remaining);
            }

            /* THE LATERAL ERROR THIS MOVE INHERITED, captured on the first
             * sweep that has a reference at all.
             *
             * It answers a question no existing log could: does a PIVOT throw
             * the robot sideways? One run came out of a dead end 46 mm further
             * from the same wall than it went in, across one 180 and one cell
             * of travel, and there was no way to tell which of the two did it.
             * The per-cycle trace only survives the last move, so this belongs
             * in the per-cell record where every move keeps one. */
            if (!sl_entry_valid && wf_side != WALL_FOLLOW_NONE) {
                sl_entry_err_mm = wf_error_mm;
                sl_entry_valid  = 1U;
            }
            last_tof = now;

            /* FRONT-WALL ALIGNMENT, applied at most once per move.
             *
             * FIRED AS LATE AS IT SAFELY CAN, which is the opposite of what it
             * used to do. The old rule looked only during the first quarter of
             * the move -- precisely when the wall is furthest and its reading
             * worst -- and the two conditions fought each other: the window is
             * at the start, the wall arrives at the end. Half the moves in a
             * measured run never aligned at all, one of them missing by six
             * millimetres of sensor reach.
             *
             * The limit on firing late is physical rather than a fraction of
             * the move: there must be room to decelerate to the new endpoint
             * from the speed the reference is actually doing. Below that, it
             * fires on the best reading available; above it, it waits for a
             * better one. */
            if (!align_tried) {

                uint16_t f = m[TOF_FRONT].distance_mm;

                if (m[TOF_FRONT].valid
                    && f != TOF_DISTANCE_INVALID
                    && f <= WALL_FRONT_ALIGN_RANGE_MM) {

                    /* Signed travel still needed for the sensor to read the
                     * target. Works in BOTH directions unchanged: driving at a
                     * wall the reading falls, so this is positive; backing away
                     * it rises, so this is negative. */
                    float to_go_cm   = ((float)f - front_target_mm) * 0.1f;
                    float new_target = measured + to_go_cm;
                    float delta      = new_target - target_cm;

                    /* What is left to travel from where the REFERENCE has
                     * reached, not from where the robot has. Anchoring on the
                     * reference is what keeps it continuous: anchoring on the
                     * robot would step the reference back by however far it
                     * currently lags, which is the same defect in a smaller
                     * size. */
                    /* TWO DIFFERENT REMAINING DISTANCES, and using the wrong
                     * one cost a run.
                     *
                     * The rebuilt profile starts where the REFERENCE is, so
                     * that is the distance it has to cover. But whether there
                     * is room to stop is a question about the ROBOT, which is
                     * behind the reference by however much it is lagging --
                     * 2 to 3 cm normally and 9 cm when it is fighting
                     * something. Asking the reference produced an alignment
                     * that refused itself on exactly the moves that were going
                     * badly, and the front-wall stops went from an 11 mm
                     * spread to 54. */
                    float remaining_ref   = new_target - ref_pos;
                    float remaining_robot = new_target - measured;

                    /* Distance needed just to come to rest from the speed the
                     * reference is doing now, plus a cushion. Taken from the
                     * live velocity rather than assumed to be cruise, because
                     * early in a move it is much less and the alignment should
                     * be allowed to fire there too. */
                    const float braking_cm =
                        (ref_vel * ref_vel)
                        / (2.0f * STRAIGHT_PROFILE_ACCEL_CMS2)
                        + WALL_FRONT_ALIGN_ROOM_CM;

                    const uint8_t has_room  =
                        (fabsf(remaining_robot) >= braking_cm);
                    const uint8_t good_read = (f <= WALL_FRONT_ALIGN_BEST_MM);

                    /* Running out of room: this is the last sweep that can
                     * still retarget, so take whatever reading is in range
                     * rather than waiting for a better one that will arrive
                     * too late to use. */
                    const uint8_t last_chance =
                        (fabsf(remaining_robot) < braking_cm
                                                  + WALL_FRONT_ALIGN_ROOM_CM);

                    /* Refuse a correction that would make the rest of the move
                     * run backwards. A reference that has already passed the
                     * new endpoint has nothing useful to do with this, and
                     * reversing is never the answer. */
                    if ((good_read || last_chance)
                        && has_room
                        && fabsf(delta) <= WALL_FRONT_ALIGN_MAX_CM
                        && remaining_ref * distance_cm > 0.0f) {

                        /* New profile from here, at the speed the reference is
                         * already doing, and a clock to match. Continuous in
                         * position AND velocity -- rebuilding from rest would
                         * drop the feedforward to zero and command a brake and
                         * a fresh start in the middle of a move the robot is
                         * already making. */
                        MotionProfile_t rebuilt;

                        if (MotionProfile_InitFrom(&rebuilt, remaining_ref, ref_vel,
                                                   STRAIGHT_PROFILE_MAX_CMS,
                                                   STRAIGHT_PROFILE_ACCEL_CMS2)) {

                            dist_profile  = rebuilt;
                            target_cm     = new_target;
                            align_base_cm = ref_pos;
                            align_t0_s    = elapsed_s;

                            sl_align_delta_cm = delta;
                            sl_align_applied  = 1U;
                            align_tried       = 1U;
                        }
                        /* Infeasible after all: leave the move alone. The
                         * has_room test should have caught it, so this is the
                         * profile generator having the last word on its own
                         * arithmetic rather than a condition worth duplicating
                         * here. */
                    }
                    /* Out of range: leave align_tried clear and look again on
                     * the next sweep. The wall may simply not be the one this
                     * move is aiming at yet. */
                }
            }
        }

        /* The cascade: lateral error tilts the HEADING TARGET, and the
         * heading loop closes on that. The drift bleed rides along on the
         * same signal. */
        float heading_target = TurnController_GetHeadingTargetDeg()
                               + WallFollow_GetDriftDeg();

        sl_yaw_target_deg = heading_target + tilt_deg;

        float yaw = YawEstimator_GetYawDeg();

        sl_yaw_error_deg = sl_yaw_target_deg - yaw;

        /* Track the profile, not the endpoint. Feedforward supplies the
         * command the move needs; feedback only trims. */
        float base = STRAIGHT_FF_GAIN * ref_vel
                     + PIDController_Update(&controller.distance_pid, ref_pos, measured);
        float steer = PIDController_Update(&yaw_pid, sl_yaw_target_deg, yaw);

        /* Stiction floor, NEVER during deceleration.
         *
         * The floor exists to raise a command too small to move the robot. It
         * must not raise a command that is deliberately small because the
         * profile is braking. The trace caught it doing exactly that: at
         * t=1512 the distance PID wanted to slow down and applyMinSpeed forced
         * the command back up to +45, driving the robot straight through the
         * target to 21.6 cm.
         *
         * Gating on reference ACCELERATION rather than velocity is what makes
         * this correct: velocity is still large during the braking ramp, which
         * is precisely when flooring is most harmful. turn_controller already
         * guards this with TURN_PROFILE_FLOOR_DPS; the straight path never got
         * the same gate. */
        float ref_acc = MotionProfile_Acceleration(&dist_profile, prof_t_s);

        /* The rule is "floor the command unless the profile is braking", and
         * braking is exactly when reference velocity and acceleration disagree
         * in sign. Written that way rather than as `ref_acc >= 0` because the
         * two are equivalent going forwards and only one of them stays true if
         * this ever has to run a move in the other direction. */
        if (arriving && !chaining) {
            /* Committed to stopping, so stop driving. Keyed on the latch and
             * not on the band, so a robot that has coasted a little past does
             * not get commanded back into it. */
            base = 0.0f;
        }
        else if (ref_acc * ref_vel >= 0.0f && fabsf(ref_vel) > 1.0f) {
            base = applyMinSpeed(base);
        }

        /* BREAKAWAY. Armed only once the profile has finished: while it is
         * still running a slow patch is the controller's problem to solve, and
         * a full-scale pulse mid-move would wreck the tracking. After it ends,
         * a robot that is outside tolerance and not moving is stuck, and no
         * amount of waiting fixes that.
         *
         * The command is applied at full scale in the direction of the
         * remaining error, briefly. It is a nudge to get the wheel over static
         * friction, after which the ordinary feedback has only kinetic
         * friction to work against. */
        const float still_threshold =
            STRAIGHT_BREAKAWAY_RATE_CMS * CONTROL_SAMPLE_TIME_S;

        const uint8_t profile_done = (prof_t_s >= MotionProfile_Duration(&dist_profile));
        const uint8_t short_of_it  = (fabsf(measured - target_cm) >= DISTANCE_TOLERANCE_CM);

        if (profile_done && short_of_it && travelled < still_threshold) {
            stall_cycles++;
        }
        else {
            stall_cycles = 0;
        }

        if (stall_cycles >= STRAIGHT_BREAKAWAY_CYCLES
            && sl_breakaway_count < STRAIGHT_BREAKAWAY_MAX) {

            pulse_until_ms = now + STRAIGHT_BREAKAWAY_MS;
            stall_cycles   = 0;
            sl_breakaway_count++;
        }

        if (now < pulse_until_ms) {
            base = (target_cm > measured) ? CONTROL_MAX_SPEED : -CONTROL_MAX_SPEED;
        }

        /* GIVE UP ON A MOVE THAT IS NOT HAPPENING.
         *
         * Distinct from the breakaway above, which only arms once the profile
         * has FINISHED. This is the case that was never covered: a robot
         * wedged in the MIDDLE of a move, command above the stiction floor and
         * therefore genuinely trying, and going nowhere. One was measured
         * grinding like that for over two seconds at 3.8 cm/s against a
         * profile asking for 10.
         *
         * Grinding costs more than the time. The lateral integral keeps
         * learning from an error it cannot fix, the reference sails away so
         * the front-wall alignment never gets its chance, and the run ends
         * looking like a steering fault rather than a mechanical one. Failing
         * the move says what actually happened. */
        if (fabsf(base) >= CONTROL_MIN_MOVE_SPEED
            && speed_cms < STRAIGHT_STALL_RATE_CMS
            && sl_breakaway_count >= STRAIGHT_BREAKAWAY_MAX) {

            /* ONLY ONCE THE BREAKAWAY HAS HAD ITS TURN. The pulse is the
             * recovery this drivetrain was given for exactly this situation,
             * and abandoning a move on a private clock could cut it off before
             * it had spent its budget. Waiting costs a bounded amount of time
             * and removes the case where the robot gives up somewhere it
             * could plainly have driven on. */
            if (stall_since_ms == 0U) {
                stall_since_ms = now;
            }
            else if (now - stall_since_ms >= STRAIGHT_STALL_ABORT_MS) {
                Motor_Brake();
                flightReset();
                sl_stall_abort = 1U;
                controller.state = STRAIGHTLINE_IDLE;
                return 0;
            }
        }
        else {
            stall_since_ms = 0U;
        }

        float left, right;
        allocate(base, steer, &left, &right);

        Motor_runSignedSpeed(left, right);

        if (tm_sl_trace_count < SL_TRACE_CAPACITY) {
            volatile StraightTrace_t *tr = &tm_sl_trace[tm_sl_trace_count];
            tr->t_s     = elapsed_s;
            tr->ref_cm  = ref_pos;
            tr->act_cm  = measured;
            tr->base    = sl_basespeed;
            tr->steer   = sl_steering;
            tr->yaw_err = sl_yaw_error_deg;
            tr->yaw_deg = yaw;
            tr->tilt_deg = tilt_deg;
            tr->drift_deg = WallFollow_GetDriftDeg();
            tr->err_mm = wf_error_mm;
            tm_sl_trace_count++;
        }
    }
}


uint8_t runForwardFused(float distance_cm)
{
    /* Aims to finish WALL_FRONT_ALIGN_MM from a wall ahead, when there is one.
     * With no wall in range the alignment never fires and the move is exactly
     * what it was before: odometry against a trapezoidal profile.
     *
     * Every option at its default -- rest to rest, no in-flight reading -- so
     * this is the move the robot has always made, expressed in the new form
     * rather than reimplemented alongside it. */
    StraightMove_t mv = {
        .distance_cm      = distance_cm,
        .front_target_mm  = WALL_FRONT_ALIGN_MM,
        .entry_speed_cms  = 0.0f,
        .exit_speed_cms   = 0.0f,
        .keep_wall_follow = 0U,
        .keep_odometry    = 0U,
        .wall_window_cm   = -1.0f,
        .wall_centre_cm   = 0.0f,
    };

    return runForwardMove(&mv);
}
