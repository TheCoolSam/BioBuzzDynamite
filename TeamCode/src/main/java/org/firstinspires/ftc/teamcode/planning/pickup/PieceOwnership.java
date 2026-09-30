package org.firstinspires.ftc.teamcode.planning.pickup;

/**
 * Who a detected piece belongs to. Stored as data so the planner never
 * branches on red versus blue. The caller sets {@link TrackedPiece}'s
 * collectable flag from the match rules. A piece marked collectable is a
 * legal target even if its ownership is {@link #OPPONENT}.
 */
public enum PieceOwnership {
    ALLIANCE,
    OPPONENT,
    NEUTRAL
}
