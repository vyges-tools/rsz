// SPDX-License-Identifier: Apache-2.0
//! The estimator's parasitics per scene, each distinct map held once.
//!
//! The reference keeps one set of parasitics per scene ("make separate parasitics for each
//! corner"), each estimated from that scene's wire RC. When two scenes' estimates are equal (the
//! same RC: a design with no per-corner `set_layer_rc`), every read of one is a read of the
//! other, so they are held as one map. A later estimate that gives two such scenes different
//! values splits them first. What a scene reads is always what its own map would hold.

use std::collections::HashMap;

use vyges_sta::graph::NetParasitics;

/// One net's parasitics by net name.
pub type NetMap = HashMap<String, NetParasitics>;

/// Per scene, its parasitics; equal scenes share one map.
#[derive(Debug, Clone, Default)]
pub struct SceneParasitics {
    maps: Vec<NetMap>,
    /// Each scene's map in `maps`.
    slot: Vec<usize>,
}

impl SceneParasitics {
    /// `per_scene` with every map equal to an earlier scene's held once.
    pub fn new(per_scene: Vec<NetMap>) -> SceneParasitics {
        let mut s = SceneParasitics::default();
        for m in per_scene {
            match s.maps.iter().position(|u| *u == m) {
                Some(at) => s.slot.push(at),
                None => {
                    s.slot.push(s.maps.len());
                    s.maps.push(m);
                }
            }
        }
        s
    }

    /// Scene `k` reads `maps[slot[k]]` (scenes given one map read alike by construction).
    pub fn from_slots(maps: Vec<NetMap>, slot: Vec<usize>) -> SceneParasitics {
        assert!(slot.iter().all(|&s| s < maps.len()), "every scene's map is one of the maps");
        SceneParasitics { maps, slot }
    }

    /// The number of scenes.
    pub fn len(&self) -> usize {
        self.slot.len()
    }

    pub fn is_empty(&self) -> bool {
        self.slot.is_empty()
    }

    /// How many distinct maps are held (a diagnostic).
    pub fn distinct(&self) -> usize {
        self.maps.len()
    }

    /// Scene `k`'s parasitics.
    pub fn scene(&self, k: usize) -> &NetMap {
        &self.maps[self.slot[k]]
    }

    /// `net`'s parasitics gone from every scene.
    pub fn remove(&mut self, net: &str) {
        for m in &mut self.maps {
            m.remove(net);
        }
    }

    /// `p` as `net`'s parasitics in every scene.
    pub fn insert_all(&mut self, net: &str, p: &NetParasitics) {
        for m in &mut self.maps {
            m.insert(net.to_string(), p.clone());
        }
    }

    /// Per scene, `net`'s parasitics (`None`: none). Two scenes sharing a map given different
    /// values are split first, so each reads its own.
    pub fn set(&mut self, net: &str, per_scene: Vec<Option<NetParasitics>>) {
        assert_eq!(per_scene.len(), self.slot.len(), "one value per scene");
        for k in 1..self.slot.len() {
            let s = self.slot[k];
            if let Some(j) = (0..k).find(|&j| self.slot[j] == s) {
                if per_scene[j] != per_scene[k] {
                    let copy = self.maps[s].clone();
                    self.slot[k] = self.maps.len();
                    self.maps.push(copy);
                }
            }
        }
        let mut written = vec![false; self.maps.len()];
        for (k, p) in per_scene.into_iter().enumerate() {
            let s = self.slot[k];
            if std::mem::replace(&mut written[s], true) {
                continue;
            }
            match p {
                Some(p) => self.maps[s].insert(net.to_string(), p),
                None => self.maps[s].remove(net),
            };
        }
    }

    /// Scene `k`'s map, taken (copied when another scene shares it, so that one keeps it).
    pub fn take_scene(&mut self, k: usize) -> NetMap {
        let s = self.slot[k];
        if self.slot.iter().filter(|&&x| x == s).count() > 1 {
            self.maps[s].clone()
        } else {
            std::mem::take(&mut self.maps[s])
        }
    }

    /// Scene `k`'s map, the rest let go.
    pub fn into_scene(mut self, k: usize) -> NetMap {
        std::mem::take(&mut self.maps[self.slot[k]])
    }

    /// One map per scene (a shared map copied for each scene after the first).
    pub fn into_scenes(mut self) -> Vec<NetMap> {
        let n = self.slot.len();
        let mut last = vec![usize::MAX; self.maps.len()];
        for k in 0..n {
            last[self.slot[k]] = k;
        }
        (0..n).map(|k| if last[self.slot[k]] == k { std::mem::take(&mut self.maps[self.slot[k]]) } else { self.maps[self.slot[k]].clone() }).collect()
    }
}

impl From<Vec<NetMap>> for SceneParasitics {
    fn from(per_scene: Vec<NetMap>) -> SceneParasitics {
        SceneParasitics::new(per_scene)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use vyges_sta::parasitics::Network;

    fn p(cap: f32) -> NetParasitics {
        NetParasitics { node_names: vec!["u1/Y".into(), "u2/A".into()], network: Network { node_caps: vec![cap, cap], resistors: vec![(0, 1, 10.0)] }, ..Default::default() }
    }

    fn map(cap: f32) -> NetMap {
        [("n1".to_string(), p(cap)), ("n2".to_string(), p(cap * 2.0))].into()
    }

    /// Rule (OpenROAD `EstimateParasitics`: one set of parasitics per scene, each from its own wire
    /// RC): every scene reads what its own map would hold — equal scenes held once, a later
    /// estimate that differs between them splitting them, never one scene's value read by another.
    #[test]
    fn each_scene_reads_its_own_parasitics_whether_shared_or_not() {
        let mut s = SceneParasitics::new(vec![map(1.0), map(1.0), map(3.0)]);
        assert_eq!((s.len(), s.distinct()), (3, 2), "scenes 0 and 1 share one map");
        assert_eq!(s.scene(1), &map(1.0));
        assert_eq!(s.scene(2), &map(3.0));
        // the same value for the shared scenes: still shared
        s.set("n1", vec![Some(p(5.0)), Some(p(5.0)), Some(p(6.0))]);
        assert_eq!(s.distinct(), 2);
        assert_eq!((s.scene(0)["n1"].network.node_caps[0], s.scene(1)["n1"].network.node_caps[0], s.scene(2)["n1"].network.node_caps[0]), (5.0, 5.0, 6.0));
        // different values for the shared scenes: split, each its own
        s.set("n2", vec![Some(p(7.0)), None, Some(p(8.0))]);
        assert_eq!(s.distinct(), 3);
        assert_eq!(s.scene(0)["n2"].network.node_caps[0], 7.0);
        assert!(!s.scene(1).contains_key("n2"), "scene 1 has none");
        assert_eq!(s.scene(1)["n1"].network.node_caps[0], 5.0, "the split kept what scene 1 held");
        s.remove("n1");
        assert!((0..3).all(|k| !s.scene(k).contains_key("n1")));
        s.insert_all("n3", &p(9.0));
        assert!((0..3).all(|k| s.scene(k)["n3"] == p(9.0)));
        let shared = SceneParasitics::new(vec![map(1.0), map(1.0)]);
        let mut t = shared.clone();
        assert_eq!(t.take_scene(0), map(1.0));
        assert_eq!(t.scene(1), &map(1.0), "a shared map taken by one scene stays the other's");
        assert_eq!(shared.clone().into_scene(1), map(1.0));
        assert_eq!(shared.into_scenes(), vec![map(1.0), map(1.0)]);
    }
}
