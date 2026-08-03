import unittest

from main.tasks.task333.shoot_target import (
    BOARD_SPACING_M,
    BoardTracker,
    select_confirmation_target,
)


def detection(xc, score=0.9, yc=0.5):
    return {
        "label": "animal",
        "score": score,
        "bbox_norm": {
            "x_center": xc,
            "y_center": yc,
            "width": 0.08,
            "height": 0.12,
        },
    }


class TargetIdentityTests(unittest.TestCase):
    def test_missing_middle_board_does_not_steal_next_board(self):
        tracker = BoardTracker(n_total=4)
        tracker.initialize([
            detection(0.20),
            detection(0.36),
            detection(0.52),
            detection(0.68),
        ])

        frame = [
            detection(0.00),
            detection(0.16),
            detection(0.48),
        ]
        matches = tracker.update(frame)

        self.assertIsNone(matches[3])
        self.assertIs(matches[4], frame[2])
        self.assertFalse(tracker.is_hit(4))

        tracker.update(frame)
        self.assertFalse(tracker.is_hit(3))
        self.assertFalse(tracker.is_hit(4))

    def test_adjacent_board_spacing_is_8cm(self):
        self.assertEqual(BOARD_SPACING_M, 0.08)

    def test_identity_mapping_preserves_global_numbers_when_starting_mid_row(self):
        tracker = BoardTracker(n_total=4)
        first = [detection(0.59), detection(0.84)]
        tracker.initialize(first, {0: 2, 1: 3})

        frame = [detection(0.48), detection(0.73), detection(0.90)]
        matches = tracker.update(frame, {2: 4})

        self.assertIsNone(tracker.first_seen_xc(1))
        self.assertIs(matches[2], frame[0])
        self.assertIs(matches[3], frame[1])
        self.assertIs(matches[4], frame[2])

    def test_hit_board_is_never_rebound(self):
        tracker = BoardTracker(n_total=4)
        tracker.initialize([
            detection(0.20),
            detection(0.36),
            detection(0.52),
            detection(0.68),
        ])
        tracker.mark_hit(3)

        frame = [
            detection(0.00),
            detection(0.16),
            detection(0.48),
        ]
        matches = tracker.update(frame)

        self.assertIsNone(matches.get(3))
        self.assertIs(matches[4], frame[2])
        self.assertTrue(tracker.is_hit(3))

    def test_single_visible_target_keeps_last_motion_model(self):
        tracker = BoardTracker(n_total=4)
        tracker.initialize([
            detection(0.20),
            detection(0.36),
            detection(0.52),
            detection(0.68),
        ])
        tracker.update([
            detection(0.00),
            detection(0.16),
            detection(0.48),
        ])

        frame = [detection(0.48)]
        matches = tracker.update(frame)

        self.assertIs(matches[4], frame[0])
        self.assertIsNone(matches[3])

    def test_confirmation_does_not_use_adjacent_board(self):
        reference_xcs = [0.20, 0.36, 0.52, 0.68]
        frame_without_target = [
            detection(0.00),
            detection(0.16),
            detection(0.48),
        ]

        selected = select_confirmation_target(
            frame_without_target,
            reference_xcs,
            target_index=2,
            anchor_xc=0.32,
        )
        self.assertIsNone(selected)

        frame_with_target = [
            detection(0.00),
            detection(0.16),
            detection(0.31),
            detection(0.48),
        ]
        selected = select_confirmation_target(
            frame_with_target,
            reference_xcs,
            target_index=2,
            anchor_xc=0.31,
        )
        self.assertIs(selected, frame_with_target[2])


if __name__ == "__main__":
    unittest.main()
