// SPDX-License-Identifier: Apache-2.0
//! The reference's sorts, where their order is a value.
//!
//! `std::sort` / `std::ranges::sort` are not stable. libstdc++'s introsort leaves any range of
//! at most 16 elements entirely to its final insertion sort, which IS stable; above 16 the
//! partitioning reorders equal elements in a way that depends on the whole input. So: up to 16
//! elements a stable sort is the same answer; above 16 it is the same answer only when no two
//! elements compare equal — and when two do, the order is refused rather than guessed.

use std::cmp::Ordering;

/// libstdc++'s `_S_threshold`: ranges this small are only insertion-sorted.
pub const INSERTION_THRESHOLD: usize = 16;

/// Two elements the comparator cannot order, in a range where the reference's sort would not
/// keep them in input order.
#[derive(Debug, Clone, PartialEq)]
pub struct UnorderedTie {
    pub len: usize,
}

/// `std::sort(v, less)` where `less` is a strict weak order, as `Ordering`.
pub fn std_sort_by<T>(v: &mut [T], mut cmp: impl FnMut(&T, &T) -> Ordering) -> Result<(), UnorderedTie> {
    v.sort_by(&mut cmp);
    if v.len() > INSERTION_THRESHOLD && v.windows(2).any(|w| cmp(&w[0], &w[1]) == Ordering::Equal) {
        return Err(UnorderedTie { len: v.len() });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    // Rule (libstdc++ std::sort): ≤ 16 elements is insertion sort — ties keep input order.
    #[test]
    fn a_short_range_keeps_ties_in_input_order() {
        let mut v = vec![(2, 'a'), (1, 'b'), (2, 'c'), (1, 'd')];
        std_sort_by(&mut v, |a, b| a.0.cmp(&b.0)).unwrap();
        assert_eq!(v, vec![(1, 'b'), (1, 'd'), (2, 'a'), (2, 'c')]);
    }

    // Rule: above 16 a tie's order depends on introsort's partitioning — refused, not guessed.
    #[test]
    fn a_long_range_with_a_tie_is_refused_and_without_one_is_sorted() {
        let mut tied: Vec<i32> = (0..17).map(|i| i / 2).collect();
        assert_eq!(std_sort_by(&mut tied, |a, b| a.cmp(b)), Err(UnorderedTie { len: 17 }));
        let mut distinct: Vec<i32> = (0..17).rev().collect();
        std_sort_by(&mut distinct, |a, b| a.cmp(b)).unwrap();
        assert_eq!(distinct, (0..17).collect::<Vec<_>>());
    }
}
