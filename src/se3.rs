use nalgebra as na;

pub type SE3 = na::Isometry3<f64>;

pub trait To7<T: na::RealField> {
    fn to7(&self) -> [T; 7];
}

impl To7<f64> for SE3 {

    fn to7(&self) -> [f64; 7] {
        let t = &self.translation;
        let q = &self.rotation;
        [t.x, t.y, t.z, q.i, q.j, q.k, q.w]
    }

}

pub fn from7(a: &[f64]) -> Option<SE3> {
    if a.len() < 7 {
        return None;
    }
    Some(
        SE3::from_parts([a[0], a[1], a[2]].into(),
        na::UnitQuaternion::from_quaternion([a[3], a[4], a[5], a[6]].into()))
    )
}

pub fn from_array(a: &[f64]) -> Option<SE3> {
    match a.len() {
        7 => Some(
            SE3::from_parts([a[0], a[1], a[2]].into(),
            na::UnitQuaternion::from_quaternion([a[3], a[4], a[5], a[6]].into()))
        ),
        // Trans only
        3 => Some(SE3::translation(a[0], a[1], a[2])),
        // Quat
        4 => Some(
            na::convert(na::UnitQuaternion::from_quaternion([a[0], a[1], a[2], a[3]].into()))
        ),
        // Rotation mat
        9 => {
            // Can't use na::convert here.
            let mat = na::Matrix3::from_row_slice(a);
            if mat.is_special_orthogonal(1e-6) {
                Some(na::convert(na::Rotation3::from_matrix_unchecked(mat)))
            } else {
                None
            }
            // Or extend to 4x4 matrix.
        }
        // SE3 mat
        16 => na::try_convert(na::Matrix4::from_row_slice(a)),
        _ => None
    }
}


#[cfg(test)]
mod test {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn pose_vec7() {
        let vec = [1.0, 2.0, -1.0, 1.0, 0.0, 0.0, 0.0];
        let pose = from_array(&vec).unwrap();
        let vec2 = pose.to7();

        assert_relative_eq!(vec.as_slice(), vec2.as_slice());
    }

    #[test]
    fn pose_conversion() {
        let pose = SE3::identity();
        assert_eq!(pose.to7(), [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]);
    }
}
