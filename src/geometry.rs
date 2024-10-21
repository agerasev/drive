use derive_more::derive::{From, Into};
use macroquad::math::{Mat3, Vec3};

#[derive(Clone, Copy, PartialEq, Debug, From, Into)]
pub struct Triangle3 {
    #[from]
    #[into]
    vs: [Vec3; 3],
}

impl Triangle3 {
    pub fn new(v0: Vec3, v1: Vec3, v2: Vec3) -> Self {
        Self { vs: [v0, v1, v2] }
    }

    pub fn vertices(&self) -> [Vec3; 3] {
        self.vs
    }

    pub fn normal(&self) -> Vec3 {
        (self.vs[1] - self.vs[0])
            .cross(self.vs[2] - self.vs[0])
            .normalize_or_zero()
    }

    /// Returns: (distance from start, intersection point, normal at the point)
    pub fn intersect_line(&self, start: Vec3, end: Vec3) -> Option<(f32, Vec3, Vec3)> {
        let length = (end - start).length();
        let dir = (end - start) / length;
        let normal = self.normal();
        let dist = (self.vs[0] - start).dot(normal) / dir.dot(normal);
        if dist >= 0.0 && dist <= length {
            let point = start + dir * dist;
            let rel_point = point - self.vs[0];
            let (u, v) = (self.vs[1] - self.vs[0], self.vs[2] - self.vs[0]);
            let uv = Mat3::from_cols(u, v, normal).inverse().mul_vec3(rel_point);
            if uv.x >= 0.0 && uv.y >= 0.0 && uv.element_sum() <= 1.0 {
                Some((dist, point, normal))
            } else {
                None
            }
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle3_intersect_line() {
        let simplex = Triangle3::new(
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 0.0),
        );
        assert_eq!(
            simplex.intersect_line(Vec3::new(0.25, 0.25, 1.23), Vec3::new(0.25, 0.25, -1.34)),
            Some((1.23, Vec3::new(0.25, 0.25, 0.0), Vec3::new(0.0, 0.0, 1.0)))
        );
        assert_eq!(
            simplex.intersect_line(Vec3::new(0.25, 0.25, -0.23), Vec3::new(0.25, 0.25, -1.34)),
            None
        );
        assert_eq!(
            simplex.intersect_line(Vec3::new(0.25, 0.25, 1.0), Vec3::new(1.0, 1.0, -1.0)),
            None
        );
    }
}
