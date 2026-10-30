package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Covers the shot-readiness gate: settle timers, hysteresis bands, the stale
 * zero-target bug, the distance gate and the chassis-motion gate.
 */
class ShotGateTest {
  private static final double DT = 0.02;

  private static ShotGate.Config config() {
    return new ShotGate.Config(
        0.03, // +/-3% flywheel speed
        0.15, // speed settle
        2.0, // aim enter tolerance
        1.5, // aim hysteresis -> exit at 3.5 deg
        0.10, // aim settle
        0.0254, // distance enter tolerance (1 in)
        0.0254, // distance hysteresis
        0.20, // max fire speed
        25.0, // max fire yaw rate
        0.20, // moving-shot threshold
        0.0); // moving-shot distance limit disabled
  }

  /** Same gate, but set up for shooting on the move out to 3.5 m. */
  private static ShotGate.Config movingConfig() {
    return new ShotGate.Config(
        0.03,
        0.15,
        2.0,
        1.5,
        0.10,
        0.0254,
        0.0254,
        3.0, // allow the chassis to be rolling
        25.0,
        0.20,
        3.5); // and only inside 3.5 m
  }

  private static ShotGate.Inputs perfect() {
    return new ShotGate.Inputs(1.0, 0.0, 0.0, 3.0, 0.0, 0.0);
  }

  /** Feeds {@code inputs} for roughly {@code seconds} and returns the last status. */
  private static ShotGate.Status hold(ShotGate gate, ShotGate.Inputs inputs, double seconds) {
    ShotGate.Status status = gate.update(DT, inputs, config());
    int steps = (int) Math.round(Math.max(0.0, seconds) / DT);
    for (int i = 0; i < steps; i++) {
      status = gate.update(DT, inputs, config());
    }
    return status;
  }

  @Test
  void speedNeedsTheSettleTimeBeforeItIsReady() {
    ShotGate gate = new ShotGate();

    assertFalse(gate.update(DT, perfect(), config()).speedReady());
    assertTrue(hold(gate, perfect(), 0.2).speedReady());
  }

  @Test
  void speedDropDropsReadinessImmediately() {
    ShotGate gate = new ShotGate();
    assertTrue(hold(gate, perfect(), 0.2).speedReady());

    ShotGate.Status slow =
        gate.update(DT, new ShotGate.Inputs(0.90, 0.0, 0.0, 3.0, 0.0, 0.0), config());
    assertFalse(slow.speedReady());
  }

  @Test
  void speedHysteresisToleratesSmallDrift() {
    ShotGate gate = new ShotGate();
    assertTrue(hold(gate, perfect(), 0.2).speedReady());

    // 4% off: outside the 3% entry band, inside the 4.5% hysteresis band.
    ShotGate.Status drifting =
        gate.update(DT, new ShotGate.Inputs(1.04, 0.0, 0.0, 3.0, 0.0, 0.0), config());
    assertTrue(drifting.speedReady());
  }

  @Test
  void zeroOrMissingTargetIsNeverReady() {
    ShotGate gate = new ShotGate();

    ShotGate.Status missing =
        hold(gate, new ShotGate.Inputs(Double.NaN, 0.0, 0.0, 3.0, 0.0, 0.0), 0.5);
    assertFalse(missing.speedReady());

    gate.reset();
    ShotGate.Status stopped =
        hold(gate, new ShotGate.Inputs(0.0, 0.0, 0.0, 3.0, 0.0, 0.0), 0.5);
    assertFalse(stopped.speedReady());
  }

  @Test
  void aimNeedsSettleAndUsesTheWideExitBand() {
    ShotGate gate = new ShotGate();

    assertFalse(gate.update(DT, perfect(), config()).aimed());
    assertTrue(hold(gate, perfect(), 0.15).aimed());

    // 3 deg: outside the 2 deg entry band, inside the 3.5 deg exit band.
    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 3.0, 0.0, 3.0, 0.0, 0.0), config()).aimed());

    // 5 deg: outside both bands, readiness drops.
    assertFalse(
        gate.update(DT, new ShotGate.Inputs(1.0, 5.0, 0.0, 3.0, 0.0, 0.0), config()).aimed());

    // 3 deg again: back inside the exit band but outside the 2 deg entry band,
    // so a fresh dwell is required.
    assertFalse(
        gate.update(DT, new ShotGate.Inputs(1.0, 3.0, 0.0, 3.0, 0.0, 0.0), config()).aimed());
    assertTrue(hold(gate, new ShotGate.Inputs(1.0, 1.5, 0.0, 3.0, 0.0, 0.0), 0.15).aimed());
  }

  @Test
  void distanceGateIsSkippedWhenTheErrorIsNaN() {
    ShotGate gate = new ShotGate();

    ShotGate.Status status =
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, Double.NaN, 3.0, 0.0, 0.0), config());
    assertTrue(status.atDistance());
  }

  @Test
  void distanceUsesHysteresisAroundTheTarget() {
    ShotGate gate = new ShotGate();

    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.01, 3.0, 0.0, 0.0), config()).atDistance());

    // 0.04 m out is inside 0.0254 + 0.0254 m hysteresis.
    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.04, 3.0, 0.0, 0.0), config()).atDistance());

    // 0.08 m out drops it.
    assertFalse(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.08, 3.0, 0.0, 0.0), config()).atDistance());

    // 0.04 m again is still outside the tight entry band.
    assertFalse(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.04, 3.0, 0.0, 0.0), config()).atDistance());
  }

  @Test
  void motionGateBlocksFastTranslationAndFastRotation() {
    ShotGate gate = new ShotGate();

    assertFalse(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, 3.0, 0.5, 0.0), config()).motionOk());
    assertFalse(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, 3.0, 0.0, 40.0), config()).motionOk());
    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, 3.0, 0.1, 10.0), config()).motionOk());
  }

  @Test
  void movingShotEnvelopeIsOffByDefault() {
    // The default config has the limit disabled, so a fast robot is judged only
    // by the flat speed limit; raising max fire speed must not silently start
    // rejecting long shots.
    ShotGate.Config fast = new ShotGate.Config(
        0.03, 0.15, 2.0, 1.5, 0.10, 0.0254, 0.0254, 3.0, 25.0, 0.20, 0.0);
    ShotGate gate = new ShotGate();

    // 1.5 m/s at 6 m: outside any sensible on-the-fly shot, still allowed
    // because the envelope is disabled.
    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, 6.0, 1.5, 0.0), fast).motionOk());
  }

  @Test
  void movingShotIsRefusedBeyondTheEnvelope() {
    ShotGate gate = new ShotGate();

    assertFalse(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, 5.0, 1.5, 0.0), movingConfig())
            .motionOk());
    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, 2.0, 1.5, 0.0), movingConfig())
            .motionOk());
  }

  @Test
  void stationaryShotIgnoresTheEnvelope() {
    ShotGate gate = new ShotGate();

    // Below the 0.20 m/s threshold the distance limit does not apply, so a long
    // shot from a standing robot is still allowed.
    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, 6.0, 0.10, 0.0), movingConfig())
            .motionOk());
  }

  @Test
  void unknownDistanceDoesNotBlockAMovingShot() {
    ShotGate gate = new ShotGate();

    assertTrue(
        gate.update(DT, new ShotGate.Inputs(1.0, 0.0, 0.0, Double.NaN, 1.5, 0.0), movingConfig())
            .motionOk());
  }

  @Test
  void resetClearsTimers() {
    ShotGate gate = new ShotGate();
    assertTrue(hold(gate, perfect(), 0.3).speedReady());

    gate.reset();
    ShotGate.Status status = gate.update(DT, perfect(), config());
    assertFalse(status.speedReady());
    assertFalse(status.aimed());
  }
}
