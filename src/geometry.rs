use glam::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Triangle3 {
    vs: [Vec3; 3],
}
impl From<[Vec3; 3]> for Triangle3 {
    fn from(vs: [Vec3; 3]) -> Self {
        Self { vs }
    }
}
impl Triangle3 {
    /// Segment intersection, including endpoints. Degenerate triangles/segments
    /// and parallel rays have no hit. Returns distance, point and oriented normal.
    pub fn intersect_line(&self, start: Vec3, end: Vec3) -> Option<(f32, Vec3, Vec3)> {
        let delta = end - start;
        let length = delta.length();
        if !length.is_finite() || length <= f32::EPSILON {
            return None;
        }
        let dir = delta / length;
        let [a, b, c] = self.vs;
        let e1 = b - a;
        let e2 = c - a;
        let p = dir.cross(e2);
        let det = e1.dot(p);
        if det.abs() <= 1e-7 * e1.length() * e2.length() {
            return None;
        }
        let t = start - a;
        let u = t.dot(p) / det;
        let q = t.cross(e1);
        let v = dir.dot(q) / det;
        let distance = e2.dot(q) / det;
        if u >= -1e-6 && v >= -1e-6 && u + v <= 1.0 + 1e-6 && (0.0..=length).contains(&distance) {
            Some((
                distance,
                start + distance * dir,
                e1.cross(e2).try_normalize()?,
            ))
        } else {
            None
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn intersections_handle_edges_parallel_and_degenerate_segments() {
        let tri = Triangle3::from([Vec3::ZERO, Vec3::X, Vec3::Y]);
        let (dist, p, n) = tri
            .intersect_line(Vec3::new(0.25, 0.25, 1.0), Vec3::new(0.25, 0.25, -1.0))
            .unwrap();
        assert_eq!(dist, 1.0);
        assert_eq!(p, Vec3::new(0.25, 0.25, 0.0));
        assert_eq!(n, Vec3::Z);
        assert!(
            tri.intersect_line(Vec3::new(0.5, 0.5, 1.0), Vec3::new(0.5, 0.5, -1.0))
                .is_some()
        );
        assert!(tri.intersect_line(Vec3::Z, Vec3::Z + Vec3::X).is_none());
        assert!(tri.intersect_line(Vec3::ZERO, Vec3::ZERO).is_none());
        assert!(
            Triangle3::from([Vec3::ZERO; 3])
                .intersect_line(Vec3::Z, -Vec3::Z)
                .is_none()
        );
    }
}
