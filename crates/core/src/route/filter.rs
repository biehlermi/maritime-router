//! Which passages a request may use.

use crate::model::{Graph, NodeId, PassageInfo};

/// Errors from building a passage filter.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum FilterError {
    /// The ID is not a passage of this data file.
    #[error("unknown passage {0:?}")]
    UnknownPassage(String),
    /// The same ID is in both `avoid` and `allow`.
    #[error("passage {0:?} is both avoided and allowed")]
    Conflict(String),
}

/// The set of allowed passage values (bit `k` = passage value `k`; value 0,
/// "no passage", is always allowed).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct PassageFilter {
    bits: [u64; 4],
}

impl PassageFilter {
    /// Every passage allowed.
    pub fn allow_all() -> Self {
        Self {
            bits: [u64::MAX; 4],
        }
    }

    /// Only water outside passages.
    pub fn none() -> Self {
        Self { bits: [1, 0, 0, 0] }
    }

    /// The passages' defaults.
    pub fn defaults(passages: &[PassageInfo]) -> Self {
        let mut f = Self::none();
        for (k, p) in passages.iter().enumerate() {
            f.set(k + 1, p.default_allowed);
        }
        f
    }

    /// The defaults overridden by a request's `avoid` and `allow` lists.
    pub fn from_request<S: AsRef<str>>(
        passages: &[PassageInfo],
        avoid: &[S],
        allow: &[S],
    ) -> Result<Self, FilterError> {
        let index = |id: &str| {
            passages
                .iter()
                .position(|p| p.id == id)
                .map(|k| k + 1)
                .ok_or_else(|| FilterError::UnknownPassage(id.to_owned()))
        };
        let mut f = Self::defaults(passages);
        for id in avoid {
            let id = id.as_ref();
            if allow.iter().any(|a| a.as_ref() == id) {
                return Err(FilterError::Conflict(id.to_owned()));
            }
            f.set(index(id)?, false);
        }
        for id in allow {
            f.set(index(id.as_ref())?, true);
        }
        Ok(f)
    }

    fn set(&mut self, value: usize, allowed: bool) {
        if value == 0 || value > 255 {
            return;
        }
        let (word, bit) = (value / 64, value % 64);
        if allowed {
            self.bits[word] |= 1 << bit;
        } else {
            self.bits[word] &= !(1 << bit);
        }
    }

    /// Whether a passage value is allowed.
    pub fn allows(&self, passage: u8) -> bool {
        let v = usize::from(passage);
        self.bits[v / 64] & (1 << (v % 64)) != 0
    }

    /// Whether a node may be used.
    pub fn allows_node(&self, graph: &Graph<'_>, n: NodeId) -> bool {
        self.allows(graph.passage(n))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn passages() -> Vec<PassageInfo> {
        vec![
            PassageInfo::new("kiel", "Kiel Canal", true),
            PassageInfo::new("northwest", "Northwest Passage", false),
        ]
    }

    #[test]
    fn defaults_and_overrides() {
        let p = passages();
        let d = PassageFilter::defaults(&p);
        assert!(d.allows(0) && d.allows(1) && !d.allows(2));
        let f = PassageFilter::from_request(&p, &["kiel"], &["northwest"]).unwrap();
        assert!(f.allows(0) && !f.allows(1) && f.allows(2));
        assert_eq!(
            PassageFilter::from_request(&p, &["suez"], &[]),
            Err(FilterError::UnknownPassage("suez".into()))
        );
        assert_eq!(
            PassageFilter::from_request(&p, &["kiel"], &["kiel"]),
            Err(FilterError::Conflict("kiel".into()))
        );
        assert!(PassageFilter::allow_all().allows(255));
        assert!(!PassageFilter::none().allows(1));
    }
}
