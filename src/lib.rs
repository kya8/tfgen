use std::io;

use itertools::Itertools;
use petgraph::{
    algo::{astar, is_cyclic_undirected}, graph::{NodeIndex, UnGraph}, visit::EdgeRef, Direction
};
use se3::SE3; // tuple_windows

pub mod se3;

#[derive(Debug, Default)]
pub struct TfGraph {
    g: G, // we might want to use GraphMap and HashMap<String, int> here.
                             // To find a node, we have to iterate through all nodes. Or use some external map/set.
}

type G = UnGraph<String, SE3>;

impl TfGraph {
    /// Create an empty graph
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a transform edge to the graph.
    ///
    /// Returns `None` if the new edge would make the graph cyclic. Self loops are cyclic.
    ///
    /// Updates existing tf edge.
    pub fn add_tf(&mut self, src: String, dst: String, tf: SE3) -> Option<()> {
        // The invariant must hold: Graph is not cyclic.
        debug_assert!(!is_cyclic_undirected(&self.g));

        let a = self.find_or_add_node(src);
        let b = self.find_or_add_node(dst);

        //let edge_new = self.g.update_edge(a, b, tf);
        // update_edge() will update b->a edge as well (for undirected). This is not what we want.
        let edge_new =
            if let Some((eid, Direction::Outgoing /* Only update if direction matches */)) = self.g.find_edge_undirected(a, b) {
                self.g[eid] = tf;
                eid
            } else {
                self.g.add_edge(a, b, tf)
            };
        if is_cyclic_undirected(&self.g) {
            // Graph can only become cyclic when both nodes are pre-existing.
            // So we only need to delete the new edge.
            self.g.remove_edge(edge_new);
            return None;
        }

        Some(())
    }

    pub fn query_tf(&self, src: &str, dst: &str) -> Option<(SE3, Vec<&str>)> {
        let (Some(src), Some(dst)) = (self.find_node(src), self.find_node(dst)) else {
            return None;
        };

        let (_, path_nodes) = astar(&self.g, src, |i| i == dst, |_| 1, |_| 0)?;
        // If src == dst, path contains 1 node, so tf is identity.
        let mut tf = SE3::identity();
        for (&a, &b) in path_nodes.iter().tuple_windows() {
            // or array_windows
            let (edge, dir) = self.g.find_edge_undirected(a, b).unwrap();
            let lhs = match dir {
                Direction::Outgoing => &self.g[edge],
                Direction::Incoming => &self.g[edge].inverse(),
            };
            tf = lhs * tf;
        }

        Some((tf, path_nodes.into_iter().map(|ix| self.g[ix].as_str()).collect()))
    }

    pub fn reset(&mut self) {
        self.g.clear();
    }

    fn find_node(&self, s: &str) -> Option<NodeIndex> {
        self.g.node_indices().find(|ix| self.g[*ix] == s)
    }

    fn find_or_add_node(&mut self, s: String) -> NodeIndex {
        self.find_node(&s).unwrap_or_else(|| self.g.add_node(s))
    }

    pub fn dump_json(&self, writer: &mut impl io::Write) -> Result<(), impl std::error::Error> {
        serde_json::to_writer_pretty(writer, &self.g)
    }

    pub fn load_json(&mut self, reader: &mut impl io::Read) -> Result<(), ()>  {
        let g: G = serde_json::from_reader(reader).map_err(|_|())?;
        if is_cyclic_undirected(&g) {
            Err(())
        }
        else {
            self.g = g;
            Ok(())
        }
    }

    pub fn nodes(&self) -> impl Iterator<Item = &str> {
        self.g.node_weights().map(|s| s.as_str())
    }

    pub fn transforms(&self) -> impl Iterator<Item = (&str, &str)> {
        self.g.edge_references()
        .map(|r| (self.g[r.source()].as_str(), self.g[r.target()].as_str()))
    }
}

#[allow(dead_code)]
pub mod error {

    pub enum Error {
        //Input,
        Io,
        Cycle,
    }

}

#[cfg(test)]
mod test {
    use approx::assert_relative_eq;
    use se3::from_array;
    use super::*;

    #[test]
    fn tf_graph() {
        let mut g = TfGraph::default();
        let ab = from_array(&[1.0, 2.0, 3.0, -0.70709538, -0.68076149, 0.04342179, -0.18626447]).unwrap();
        let ac = from_array(&[0.0, -2.1, 5.0, -0.20034685, -0.76316815, 0.26488707, 0.55431973]).unwrap();
        let bc = ac * ab.inverse();
        g.add_tf("a".to_owned(), "b".to_owned(), ab).unwrap();
        g.add_tf("a".to_owned(), "c".to_owned(), ac).unwrap();
        g.add_tf("x".to_owned(), "y".to_owned(), SE3::identity()).unwrap();

        assert!(g.nodes().eq(["a", "b", "c", "x", "y"].into_iter()));
        assert!(g.transforms().eq([("a", "b"), ("a", "c"), ("x", "y")].into_iter()));

        // detect cycles
        assert!(g.add_tf("b".to_owned(), "c".to_owned(), bc.clone()).is_none());

        let (bc_q, bc_path) = g.query_tf("b", "c").unwrap();
        assert_relative_eq!(bc_q, bc);
        assert_eq!(bc_path, ["b", "a", "c"]);
        // Not connected
        assert!(g.query_tf("a", "x").is_none());
    }

    #[test]
    fn large_tree() {
        let mut g = TfGraph::new();
        for i in 1..4096 { // A complete binary tree.
            let parent = (i - 1) / 2;
            g.add_tf(i.to_string(), parent.to_string(), se3::random()).unwrap();
        }

        g.query_tf("0", "4000").unwrap();
        g.query_tf("2048", "4095").unwrap();
    }
}
