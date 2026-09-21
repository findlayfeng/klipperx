//! The trapezoidal velocity queue.
//!
//! Upstream's `chelper/trapq.c`: a list of constant-acceleration segments,
//! indexed by print time, that the step generators read. A move planned by
//! [`Move`](super::plan::Move) is appended as up to three segments
//! (accelerate, cruise, decelerate); [`MoveSegment::coord`] then gives the
//! toolhead position at any time inside one.
//!
//! The C version is a doubly linked list with head and tail sentinels. Here it
//! is a `VecDeque`: the sentinels exist only to find the last segment and to
//! notice a time gap, which the deque's back does directly.

use std::collections::VecDeque;

use crate::core::klippy::mathutil::Xyz;

/// A time past any real move, so the tail sentinel is always searched past
/// (`NEVER_TIME`, `chelper/trapq.c:36`).
const NEVER_TIME: f64 = 9999999999999999.9;

/// The longest filler the very first segment may insert
/// (`MAX_NULL_MOVE`, `chelper/trapq.c:104`).
const MAX_NULL_MOVE: f64 = 1.0;

/// One constant-acceleration segment of a move.
///
/// This is upstream's `struct move` (`chelper/trapq.h:16-23`): at `print_time`
/// the toolhead is at `start_pos` moving at `start_v`, and it accelerates at
/// `2 * half_accel` (kept halved because the distance formula multiplies it by
/// `t`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveSegment {
    /// Print time the segment starts at.
    pub print_time: f64,
    /// How long it lasts.
    pub move_t: f64,
    /// Velocity at the start, mm/s.
    pub start_v: f64,
    /// Half the acceleration, mm/s^2 / 2.
    pub half_accel: f64,
    /// Position at `print_time`.
    pub start_pos: Xyz,
    /// Direction cosines: distance travelled maps to each axis through these.
    pub axes_r: Xyz,
}

impl MoveSegment {
    /// A filler that holds position (a "null move" in `trapq_add_move`).
    pub fn stationary(print_time: f64, move_t: f64, start_pos: Xyz) -> Self {
        Self {
            print_time,
            move_t,
            start_v: 0.0,
            half_accel: 0.0,
            start_pos,
            axes_r: Xyz::default(),
        }
    }

    /// How far the toolhead has moved `move_time` seconds into the segment
    /// (`move_get_distance`, `chelper/trapq.c:25-29`).
    pub fn distance(&self, move_time: f64) -> f64 {
        (self.start_v + self.half_accel * move_time) * move_time
    }

    /// The toolhead position `move_time` seconds into the segment
    /// (`move_get_coord`, `chelper/trapq.c:32-41`).
    pub fn coord(&self, move_time: f64) -> Xyz {
        let distance = self.distance(move_time);
        Xyz::new(
            self.start_pos.x() + self.axes_r.x() * distance,
            self.start_pos.y() + self.axes_r.y() * distance,
            self.start_pos.z() + self.axes_r.z() * distance,
        )
    }

    /// Whether the segment neither moves nor accelerates (a filler or a
    /// position marker).
    pub fn is_stationary(&self) -> bool {
        self.start_v == 0.0 && self.half_accel == 0.0
    }

    /// The print time the segment ends at.
    pub fn end_time(&self) -> f64 {
        self.print_time + self.move_t
    }
}

/// A trapezoidal velocity queue.
#[derive(Debug, Default)]
pub struct Trapq {
    moves: VecDeque<MoveSegment>,
    /// Segments that have finished, newest first (`trapq.c`'s history list).
    history: VecDeque<MoveSegment>,
}

impl Trapq {
    /// An empty queue.
    pub fn new() -> Self {
        Self::default()
    }

    /// The queued (not yet finished) segments, oldest first.
    pub fn moves(&self) -> &VecDeque<MoveSegment> {
        &self.moves
    }

    /// The finished segments, newest first.
    pub fn history(&self) -> &VecDeque<MoveSegment> {
        &self.history
    }

    /// Append one move's trapezoid, starting at `print_time`
    /// (`trapq_append`, `chelper/trapq.c:118-164`).
    ///
    /// The three phases become up to three segments; a phase with zero time is
    /// skipped.
    #[allow(clippy::too_many_arguments)]
    pub fn append(
        &mut self,
        print_time: f64,
        accel_t: f64,
        cruise_t: f64,
        decel_t: f64,
        start_pos: Xyz,
        axes_r: Xyz,
        start_v: f64,
        cruise_v: f64,
        accel: f64,
    ) {
        let mut print_time = print_time;
        let mut start_pos = start_pos;
        if accel_t != 0.0 {
            let segment = MoveSegment {
                print_time,
                move_t: accel_t,
                start_v,
                half_accel: 0.5 * accel,
                start_pos,
                axes_r,
            };
            self.add_segment(segment);
            print_time += accel_t;
            start_pos = segment.coord(accel_t);
        }
        if cruise_t != 0.0 {
            let segment = MoveSegment {
                print_time,
                move_t: cruise_t,
                start_v: cruise_v,
                half_accel: 0.0,
                start_pos,
                axes_r,
            };
            self.add_segment(segment);
            print_time += cruise_t;
            start_pos = segment.coord(cruise_t);
        }
        if decel_t != 0.0 {
            self.add_segment(MoveSegment {
                print_time,
                move_t: decel_t,
                start_v: cruise_v,
                half_accel: -0.5 * accel,
                start_pos,
                axes_r,
            });
        }
    }

    /// Queue a segment, filling a time gap with a stationary one
    /// (`trapq_add_move`, `chelper/trapq.c:107-129`).
    fn add_segment(&mut self, segment: MoveSegment) {
        let (prev_time, prev_end) = match self.moves.back() {
            Some(prev) => (prev.print_time, prev.end_time()),
            // Upstream's head sentinel sits at -1.0 (`trapq_alloc`).
            None => (-1.0, -1.0),
        };
        if prev_end < segment.print_time {
            let start = if prev_time <= 0.0 && segment.print_time > MAX_NULL_MOVE {
                // Bound the very first filler so the numbers stay stable.
                segment.print_time - MAX_NULL_MOVE
            } else {
                prev_end
            };
            self.moves.push_back(MoveSegment::stationary(
                start,
                segment.print_time - start,
                segment.start_pos,
            ));
        }
        self.moves.push_back(segment);
    }

    /// Move the finished segments to the history, and drop history older than
    /// `clear_history_time` (`trapq_finalize_moves`, `chelper/trapq.c:167-196`).
    pub fn finalize_moves(&mut self, print_time: f64, clear_history_time: f64) {
        while let Some(front) = self.moves.front() {
            if front.end_time() > print_time {
                break;
            }
            let segment = self.moves.pop_front().expect("checked");
            if !segment.is_stationary() {
                self.history.push_front(segment);
            }
        }
        // Keep at least the newest entry, as the C list's `latest` check does.
        while self.history.len() > 1 {
            let Some(oldest) = self.history.back() else {
                break;
            };
            if oldest.end_time() > clear_history_time {
                break;
            }
            self.history.pop_back();
        }
    }

    /// Flush every queued move and note a position change
    /// (`trapq_set_position`, `chelper/trapq.c:199-223`).
    pub fn set_position(&mut self, print_time: f64, pos: Xyz) {
        self.finalize_moves(NEVER_TIME, 0.0);
        // Drop history newer than the change, truncating the one it interrupts.
        while let Some(front) = self.history.front() {
            if front.print_time >= print_time {
                self.history.pop_front();
                continue;
            }
            if front.end_time() > print_time {
                let start = front.print_time;
                if let Some(front) = self.history.front_mut() {
                    front.move_t = print_time - start;
                }
            }
            break;
        }
        self.history
            .push_front(MoveSegment::stationary(print_time, 0.0, pos));
    }

    /// The queued and recent segments overlapping `start_time..end_time`,
    /// newest first (`trapq_extract_old`, `chelper/trapq.c:249-273`).
    pub fn extract_old(&self, max: usize, start_time: f64, end_time: f64) -> Vec<MoveSegment> {
        let mut out = Vec::new();
        for segment in self.moves.iter().rev() {
            if start_time >= segment.end_time() || out.len() >= max {
                break;
            }
            if end_time <= segment.print_time || segment.is_stationary() {
                continue;
            }
            out.push(*segment);
        }
        for segment in &self.history {
            if start_time >= segment.end_time() || out.len() >= max {
                break;
            }
            if end_time <= segment.print_time {
                continue;
            }
            out.push(*segment);
        }
        out
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_a_trapezoid_becomes_three_segments() {
        let mut trapq = Trapq::new();

        trapq.append(
            0.0,
            0.1,
            0.2,
            0.1,
            Xyz::new(0.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            0.0,
            100.0,
            1000.0,
        );

        let moves: Vec<_> = trapq.moves().iter().copied().collect();
        // Upstream's head sentinel sits at -1.0, so a stationary filler leads
        // the first real move (it holds position from -1.0 to 0.0).
        assert_eq!(moves.len(), 4);
        assert!(moves[0].is_stationary());
        assert_eq!(moves[1].print_time, 0.0);
        assert_eq!(moves[1].move_t, 0.1);
        assert_eq!(moves[1].half_accel, 500.0);
        assert_eq!(moves[2].move_t, 0.2);
        assert_eq!(moves[2].half_accel, 0.0);
        assert_eq!(moves[3].move_t, 0.1);
        assert_eq!(moves[3].half_accel, -500.0);
    }

    #[test]
    fn test_the_phases_are_continuous() {
        let mut trapq = Trapq::new();
        trapq.append(
            0.0,
            0.1,
            0.2,
            0.1,
            Xyz::new(0.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            0.0,
            100.0,
            1000.0,
        );

        let moves: Vec<_> = trapq.moves().iter().copied().collect();
        // Each phase starts where the previous ended.
        for pair in moves.windows(2) {
            assert!((pair[0].end_time() - pair[1].print_time).abs() < 1e-12);
            let end = pair[0].coord(pair[0].move_t);
            assert!((end.x() - pair[1].start_pos.x()).abs() < 1e-9);
        }
        // The whole trapezoid travels 0.1*50 + 0.2*100 + 0.1*50 = 30 mm.
        let last = moves.last().expect("three segments");
        assert!((last.coord(last.move_t).x() - 30.0).abs() < 1e-9);
    }

    #[test]
    fn test_a_time_gap_gets_a_stationary_filler() {
        let mut trapq = Trapq::new();
        trapq.append(
            0.0,
            0.1,
            0.0,
            0.0,
            Xyz::new(0.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            0.0,
            10.0,
            100.0,
        );

        // The next move starts 0.9 s after the first ends.
        trapq.append(
            1.0,
            0.1,
            0.0,
            0.0,
            Xyz::new(1.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            0.0,
            10.0,
            100.0,
        );

        let moves: Vec<_> = trapq.moves().iter().copied().collect();
        // The leading filler (from the -1.0 sentinel), the first move, the gap
        // filler, and the second move.
        assert_eq!(moves.len(), 4);
        assert!(moves[2].is_stationary());
        assert!((moves[2].print_time - 0.1).abs() < 1e-12);
        assert!((moves[2].end_time() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_coord_follows_the_acceleration() {
        // A pure accelerating segment: starts at rest, accelerates at 100.
        let segment = MoveSegment {
            print_time: 0.0,
            move_t: 1.0,
            start_v: 0.0,
            half_accel: 50.0,
            start_pos: Xyz::new(0.0, 0.0, 0.0),
            axes_r: Xyz::new(1.0, 0.0, 0.0),
        };

        // distance = (0 + 50 * .5) * .5
        assert!((segment.distance(0.5) - 12.5).abs() < 1e-12);
        assert!((segment.coord(0.5).x() - 12.5).abs() < 1e-12);
        // 1 s in: (0 + 50 * 1) * 1 = 50.
        assert!((segment.coord(1.0).x() - 50.0).abs() < 1e-12);
    }

    #[test]
    fn test_finalize_moves_expires_finished_segments() {
        let mut trapq = Trapq::new();
        trapq.append(
            0.0,
            0.1,
            0.0,
            0.0,
            Xyz::new(0.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            0.0,
            10.0,
            100.0,
        );

        trapq.finalize_moves(0.1, 0.0);

        assert!(trapq.moves().is_empty());
        assert_eq!(trapq.history().len(), 1);
    }

    #[test]
    fn test_extract_old_returns_the_segments_in_a_window() {
        let mut trapq = Trapq::new();
        trapq.append(
            0.0,
            0.1,
            0.0,
            0.0,
            Xyz::new(0.0, 0.0, 0.0),
            Xyz::new(1.0, 0.0, 0.0),
            0.0,
            10.0,
            100.0,
        );

        let window = trapq.extract_old(10, 0.0, 0.1);

        assert_eq!(window.len(), 1);
        assert_eq!(window[0].print_time, 0.0);
    }
}
