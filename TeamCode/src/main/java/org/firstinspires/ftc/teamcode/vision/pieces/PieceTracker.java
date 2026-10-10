package org.firstinspires.ftc.teamcode.vision.pieces;

import org.firstinspires.ftc.teamcode.planning.pickup.BallType;
import org.firstinspires.ftc.teamcode.planning.pickup.PieceOwnership;
import org.firstinspires.ftc.teamcode.planning.pickup.TrackedPiece;
import org.firstinspires.ftc.teamcode.state.RobotState;
import org.firstinspires.ftc.teamcode.state.RobotStateHistory;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;

/**
 * Associates floor projections with stable physical ids.
 *
 * <p>Each update projects observations, then matches them to existing tracks
 * of the same {@link BallType}. The match is a one-to-one assignment that
 * minimizes total squared field error inside the gate. A second detection is
 * not merged into a track that already took an observation. Unmatched
 * observations start tracks. Unmatched tracks age out. There is no motion
 * model. The HuskyLens block index is never used as {@link TrackedPiece#getId()}.
 *
 * <p>{@link RobotStateHistory} supplies the acquisition-time pose at
 * {@code observationTime - configuredLatency}. Latency defaults to 0 until
 * it is measured. A missing historical pose does not become a field piece.
 * Observation timestamps are kept. This class does not invent a latency
 * number and does not store pose history itself.
 *
 * <p>An invalid pose produces no planner pieces. Tracks are still aged so a
 * long outage can expire them, and new field positions are not invented.
 */
public final class PieceTracker {

    private static final double CONSISTENCY_WEIGHT = 0.25;
    private static final double MAX_CAPTURE_POSE_AGE_SEC=.2;

    private final FloorProjection projection;
    private final PieceTrackerConstants constants;
    private final List<Track> tracks = new ArrayList<Track>();
    private int nextId = 1;
    private double lastFrameSec = Double.NEGATIVE_INFINITY;
    private double resetTimeSec = Double.NEGATIVE_INFINITY;
    private long generation = Long.MIN_VALUE;
    private final List<Suppression> consumed = new ArrayList<Suppression>();
    public static final int MAX_TRACKS = 32;
    public static final int MAX_OBSERVATIONS = 6;

    /** Clear coordinates and pending evidence after a pose reset; ids are never reused. */
    public void reset(double nowSec, long resetGeneration) {
        if (!Double.isFinite(nowSec)) throw new IllegalArgumentException("Finite reset time required");
        tracks.clear(); consumed.clear(); lastFrameSec = nowSec;
        resetTimeSec = nowSec; generation = resetGeneration;
    }

    /** Confirmed collection, with a short spatial tombstone for delayed camera frames. */
    public boolean consume(int id, double nowSec) {
        if (!Double.isFinite(nowSec)) return false;
        for (Iterator<Track> it = tracks.iterator(); it.hasNext();) {
            Track track = it.next();
            if (track.id == id) {
                if (consumed.size() == MAX_TRACKS) consumed.remove(0);
                consumed.add(new Suppression(track.fieldX, track.fieldY, nowSec));
                it.remove(); return true;
            }
        }
        return false;
    }

    public int size() { return tracks.size(); }

    public PieceTracker(FloorProjection projection, PieceTrackerConstants constants) {
        this.projection = projection == null ? HomographyFloorProjection.unconfigured() : projection;
        this.constants = constants == null ? PieceTrackerConstants.defaults() : constants;
    }

    public PieceTrackingResult update(
            List<PieceObservation> observations,
            RobotStateHistory history,
            double nowSec) {
        double now = nowSec;
        if (!Double.isFinite(now)) return new PieceTrackingResult(new ArrayList<TrackedPiece>(),
                new ArrayList<PieceTrackingResult.Sighting>(), projection.isConfigured(), false, 0, 0);
        expire(now);
        consumed.removeIf(s -> now > s.time + constants.trackTimeoutSec);
        boolean calibrated = projection.isConfigured();
        int rawCount = observations == null ? 0 : observations.size();
        int ignored = 0;

        List<PieceTrackingResult.Sighting> sightings = new ArrayList<PieceTrackingResult.Sighting>();
        List<Candidate> candidates = new ArrayList<Candidate>();

        if (observations != null) {
            for (int i = 0; i < Math.min(observations.size(), MAX_OBSERVATIONS); i++) {
                PieceObservation observation = observations.get(i);
                if (observation == null || !observation.isValid()) {
                    continue;
                }
                PieceTrackerConstants.ClassifiedPiece classified = constants.classify(observation.getCameraId());
                if (classified == null) {
                    ignored++;
                    continue;
                }
                double[] pixel = anchor(observation);
                FloorPoint floor = calibrated ? projection.project(pixel[0], pixel[1]) : null;
                Double robotX = null;
                Double robotY = null;
                if (floor != null) {
                    robotX = Double.valueOf(floor.getXForwardInches());
                    robotY = Double.valueOf(floor.getYLeftInches());
                }
                double readTime = observation.getTimestampSec();
                if (!Double.isFinite(readTime) || readTime > now + 1e-9
                        || now - readTime > constants.trackTimeoutSec
                        || readTime <= resetTimeSec || readTime <= lastFrameSec) continue;
                double captureTime = Double.isFinite(readTime)
                        ? readTime - constants.huskyLensLatencySec
                        : now - constants.huskyLensLatencySec;
                Optional<RobotState> capture = history == null
                        ? Optional.<RobotState>empty()
                        : history.sampleAt(captureTime);
                RobotState pose = capture.isPresent() ? capture.get() : null;
                sightings.add(new PieceTrackingResult.Sighting(
                        observation.getCameraId(),
                        classified.getType(),
                        observation.getImageX(),
                        observation.getImageY(),
                        robotX,
                        robotY,
                        readTime,
                        captureTime,
                        pose));
                if (floor == null || pose == null || !pose.isFresh(captureTime,MAX_CAPTURE_POSE_AGE_SEC)) {
                    continue;
                }
                if (generation == Long.MIN_VALUE) generation = pose.getResetGeneration();
                if (generation != pose.getResetGeneration()) {
                    reset(now, pose.getResetGeneration());
                    candidates.clear(); break;
                }
                double fieldX = FieldPlacement.fieldX(
                        pose.getFieldX(), pose.getHeadingRad(),
                        floor.getXForwardInches(), floor.getYLeftInches());
                double fieldY = FieldPlacement.fieldY(
                        pose.getFieldY(), pose.getHeadingRad(),
                        floor.getXForwardInches(), floor.getYLeftInches());
                if (!Double.isFinite(fieldX) || !Double.isFinite(fieldY)) {
                    continue;
                }
                boolean suppressed = false;
                for (Suppression s : consumed) {
                    if (Math.hypot(fieldX - s.x, fieldY - s.y) <= constants.associationGateInches
                            && readTime <= s.time + constants.trackTimeoutSec) suppressed = true;
                }
                if (suppressed) continue;
                candidates.add(new Candidate(
                        classified.getType(),
                        classified.getOwnership(),
                        classified.isCollectable(),
                        floor.getXForwardInches(),
                        floor.getYLeftInches(),
                        fieldX,
                        fieldY,
                        Double.isFinite(readTime) ? readTime : now));
            }
        }

        boolean poseUsed = !candidates.isEmpty();
        for (Candidate c : candidates) lastFrameSec = Math.max(lastFrameSec, c.seenAt);
        if (!calibrated) {
            for (int i = 0; i < tracks.size(); i++) {
                tracks.get(i).matched = false;
            }
            expire(now);
            return new PieceTrackingResult(
                    new ArrayList<TrackedPiece>(),
                    sightings,
                    false,
                    false,
                    rawCount,
                    ignored);
        }
        associate(candidates, now);
        expire(now);
        return new PieceTrackingResult(
                publish(now),
                sightings,
                true,
                poseUsed,
                rawCount,
                ignored);
    }

    private void associate(List<Candidate> candidates, double now) {
        for (int i = 0; i < tracks.size(); i++) {
            tracks.get(i).matched = false;
        }
        boolean[] used = new boolean[candidates.size()];
        BallType[] types = BallType.values();
        PieceOwnership[] owners = PieceOwnership.values();
        for (int typeIndex = 0; typeIndex < types.length; typeIndex++) {
            for (int ownerIndex = 0; ownerIndex < owners.length; ownerIndex++) {
                int[] trackIndex = indexesOf(types[typeIndex], owners[ownerIndex]);
                int[] observationIndex = indexesOfObservations(
                        candidates, types[typeIndex], owners[ownerIndex]);
                int[] match = assign(trackIndex, observationIndex, candidates);
            for (int local = 0; local < observationIndex.length; local++) {
                if (match[local] < 0) {
                    continue;
                }
                int candidateIndex = observationIndex[local];
                Track track = tracks.get(trackIndex[match[local]]);
                Candidate candidate = candidates.get(candidateIndex);
                double distance = Math.hypot(
                        track.fieldX - candidate.fieldX,
                        track.fieldY - candidate.fieldY);
                apply(track, candidate, distance, now);
                used[candidateIndex] = true;
            }
            }
        }
        for (int c = 0; c < candidates.size(); c++) {
            if (!used[c] && tracks.size() < MAX_TRACKS) {
                tracks.add(create(candidates.get(c), now));
            }
        }
    }

    /**
     * One-to-one assignment for a single ball type. Among assignments that
     * match as many gated pairs as possible, this keeps the one with the
     * smallest sum of squared field errors. HuskyLens publishes at most six
     * blocks; dynamic programming bounds assignment work by the six-block cap.
     */
    private int[] assign(int[] trackIndex, int[] observationIndex, List<Candidate> candidates) {
        int observationCount = observationIndex.length;
        // Dynamic programming over observation subsets: O(tracks * 2^6 * 6).
        int masks = 1 << observationCount;
        double[] costs = new double[masks];
        java.util.Arrays.fill(costs, Double.POSITIVE_INFINITY);
        costs[0] = 0;
        int[][] assignments = new int[masks][observationCount];
        for (int[] row : assignments) java.util.Arrays.fill(row, -1);
        for (int t = 0; t < trackIndex.length; t++) {
            double[] next = costs.clone();
            int[][] nextAssignments = new int[masks][];
            for (int mask = 0; mask < masks; mask++) nextAssignments[mask] = assignments[mask].clone();
            Track track = tracks.get(trackIndex[t]);
            for (int mask = 0; mask < masks; mask++) {
                if (!Double.isFinite(costs[mask])) continue;
                for (int o = 0; o < observationCount; o++) {
                    if ((mask & (1 << o)) != 0) continue;
                    Candidate candidate = candidates.get(observationIndex[o]);
                    double dx = track.fieldX - candidate.fieldX, dy = track.fieldY - candidate.fieldY;
                    double distance = dx * dx + dy * dy;
                    if (distance > constants.associationGateInches * constants.associationGateInches) continue;
                    int nextMask = mask | (1 << o);
                    if (costs[mask] + distance < next[nextMask] - 1e-9) {
                        next[nextMask] = costs[mask] + distance;
                        nextAssignments[nextMask] = assignments[mask].clone();
                        nextAssignments[nextMask][o] = t;
                    }
                }
            }
            costs = next; assignments = nextAssignments;
        }
        int selected = 0;
        for (int mask = 1; mask < masks; mask++) {
            if (Double.isFinite(costs[mask]) && (Integer.bitCount(mask) > Integer.bitCount(selected)
                    || (Integer.bitCount(mask) == Integer.bitCount(selected) && costs[mask] < costs[selected] - 1e-9))) {
                selected = mask;
            }
        }
        return assignments[selected];
    }

    private int[] indexesOf(BallType type, PieceOwnership ownership) {
        int count = 0;
        for (int i = 0; i < tracks.size(); i++) {
            Track track = tracks.get(i);
            if (track.type == type && track.ownership == ownership) {
                count++;
            }
        }
        int[] indexes = new int[count];
        int write = 0;
        for (int i = 0; i < tracks.size(); i++) {
            Track track = tracks.get(i);
            if (track.type == type && track.ownership == ownership) {
                indexes[write++] = i;
            }
        }
        return indexes;
    }

    private static int[] indexesOfObservations(
            List<Candidate> candidates,
            BallType type,
            PieceOwnership ownership) {
        int count = 0;
        for (int i = 0; i < candidates.size(); i++) {
            Candidate candidate = candidates.get(i);
            if (candidate.type == type && candidate.ownership == ownership) {
                count++;
            }
        }
        int[] indexes = new int[count];
        int write = 0;
        for (int i = 0; i < candidates.size(); i++) {
            Candidate candidate = candidates.get(i);
            if (candidate.type == type && candidate.ownership == ownership) {
                indexes[write++] = i;
            }
        }
        return indexes;
    }

    private Track create(Candidate candidate, double now) {
        Track track = new Track();
        track.id = nextId++;
        track.type = candidate.type;
        track.ownership = candidate.ownership;
        track.collectable = candidate.collectable;
        track.robotX = candidate.robotX;
        track.robotY = candidate.robotY;
        track.fieldX = candidate.fieldX;
        track.fieldY = candidate.fieldY;
        track.hits = 1;
        track.lastSeenSec = candidate.seenAt;
        track.lastEvidenceSec = candidate.seenAt;
        track.matched = true;
        track.matchDistance = 0.0;
        track.confidence = confidence(track, now, 0.0);
        return track;
    }

    private void apply(Track track, Candidate candidate, double distance, double now) {
        if (candidate.seenAt <= track.lastSeenSec) return;
        double blend = constants.positionBlend;
        track.robotX = blend(track.robotX, candidate.robotX, blend);
        track.robotY = blend(track.robotY, candidate.robotY, blend);
        track.fieldX = blend(track.fieldX, candidate.fieldX, blend);
        track.fieldY = blend(track.fieldY, candidate.fieldY, blend);
        track.ownership = candidate.ownership;
        track.collectable = candidate.collectable;
        // Read identity is unavailable on HuskyLens; measured cadence bounds evidence growth.
        if (candidate.seenAt - track.lastEvidenceSec >= constants.evidenceIntervalSec() - 1e-9) {
            track.hits++;
            track.lastEvidenceSec = candidate.seenAt;
        }
        track.lastSeenSec = candidate.seenAt;
        track.matched = true;
        track.matchDistance = distance;
        track.confidence = confidence(track, now, distance);
    }

    private void expire(double now) {
        Iterator<Track> iterator = tracks.iterator();
        while (iterator.hasNext()) {
            Track track = iterator.next();
            double age = now - track.lastSeenSec;
            if (!Double.isFinite(age) || age < 0 || age > constants.trackTimeoutSec) {
                iterator.remove();
                continue;
            }
            track.confidence = confidence(track, now, -1.0);
        }
    }

    private List<TrackedPiece> publish(double now) {
        List<TrackedPiece> pieces = new ArrayList<TrackedPiece>();
        for (int i = 0; i < tracks.size(); i++) {
            Track track = tracks.get(i);
            double age = now - track.lastSeenSec;
            if (!Double.isFinite(age) || age < 0 || age > constants.trackTimeoutSec) {
                continue;
            }
            pieces.add(new TrackedPiece(
                    track.id,
                    track.type,
                    track.ownership,
                    track.fieldX,
                    track.fieldY,
                    track.confidence,
                    track.lastSeenSec,
                    track.collectable));
        }
        return pieces;
    }

    private double confidence(Track track, double now, double matchDistance) {
        double maturity = Math.min(1.0, track.hits / constants.matureHitCount);
        double age = Math.max(0.0, now - track.lastSeenSec);
        double freshness = constants.trackTimeoutSec <= 0.0
                ? 1.0
                : 1.0 - (age / constants.trackTimeoutSec);
        if (freshness < 0.0) {
            freshness = 0.0;
        }
        double consistency = 1.0;
        if (matchDistance >= 0.0 && constants.associationGateInches > 0.0) {
            double fraction = Math.min(1.0, matchDistance / constants.associationGateInches);
            consistency = 1.0 - (CONSISTENCY_WEIGHT * fraction);
        }
        return clamp01(maturity * freshness * consistency);
    }

    private double[] anchor(PieceObservation observation) {
        double x = observation.getImageX();
        double y = observation.getImageY();
        if (constants.anchor == ObservationAnchor.LOWER_CENTER) {
            y += observation.getImageHeight() / 2.0;
        }
        return new double[] { x, y };
    }

    private static final class Track {
        private int id;
        private BallType type;
        private PieceOwnership ownership = PieceOwnership.NEUTRAL;
        private boolean collectable;
        private double robotX;
        private double robotY;
        private double fieldX;
        private double fieldY;
        private int hits;
        private double lastSeenSec;
        private double lastEvidenceSec;
        private double confidence;
        private double matchDistance;
        private boolean matched;
    }

    private static final class Suppression {
        final double x, y, time;
        Suppression(double x, double y, double time) { this.x = x; this.y = y; this.time = time; }
    }

    private static final class Candidate {
        private final BallType type;
        private final PieceOwnership ownership;
        private final boolean collectable;
        private final double robotX;
        private final double robotY;
        private final double fieldX;
        private final double fieldY;
        private final double seenAt;

        private Candidate(
                BallType type,
                PieceOwnership ownership,
                boolean collectable,
                double robotX,
                double robotY,
                double fieldX,
                double fieldY,
                double seenAt) {
            this.type = type;
            this.ownership = ownership;
            this.collectable = collectable;
            this.robotX = robotX;
            this.robotY = robotY;
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.seenAt = seenAt;
        }
    }

    private static double blend(double previous, double sample, double alpha) {
        return (previous * (1.0 - alpha)) + (sample * alpha);
    }

    private static double clamp01(double value) {
        if (!Double.isFinite(value) || value <= 0.0) {
            return 0.0;
        }
        if (value >= 1.0) {
            return 1.0;
        }
        return value;
    }



}
