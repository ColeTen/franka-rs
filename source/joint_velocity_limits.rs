//! Position-dependent joint velocity limits read from the robot's URDF.

use roxmltree::{Document, Node};

use crate::constants::NUM_JOINTS;
use crate::errors::{FrankaError, FrankaResult};
use crate::rate_limiting::JOINT_VELOCITY_LIMITS_TOLERANCE;

/// The URDF values that determine one joint's velocity limits.
#[derive(Debug, Clone, Copy, Default)]
struct JointLimitConstants {
    /// Absolute maximum velocity in rad/s (`<limit velocity>`).
    max_velocity: f64,
    /// Velocity offset in rad/s (`<position_based_velocity_limits velocity_offset>`).
    velocity_offset: f64,
    /// Deceleration limit in rad/s² (`<position_based_velocity_limits deceleration_limit>`).
    deceleration_limit: f64,
    /// Upper position limit in rad (`<limit upper>`).
    upper_position_limit: f64,
    /// Lower position limit in rad (`<limit lower>`).
    lower_position_limit: f64,
}

/// Velocity limits for all seven joints, which shrink as a joint approaches its position limits
/// so that it can always decelerate to rest before reaching them.
///
/// The default has every constant zero, which is what robots whose URDF carries no velocity
/// limits (mobile robots) use.
#[derive(Debug, Clone, Default)]
pub struct JointVelocityLimits {
    /// Limit constants indexed by joint (joint 1 at index 0).
    joints: [JointLimitConstants; NUM_JOINTS],
}

impl JointVelocityLimits {
    /// Reads the limits from the `<limit>` and `<position_based_velocity_limits>` elements of the
    /// joints named `joint1` to `joint7` (or whose names contain those names).
    ///
    /// # Errors
    /// Returns [`FrankaError::Model`] if the document is not valid XML, has no `<robot>` root, or
    /// any of the seven joints is missing, lacks either element, or lacks a required attribute.
    pub fn from_urdf(urdf: &str) -> FrankaResult<Self> {
        let document = Document::parse(urdf)
            .map_err(|error| model_error(format!("invalid URDF XML: {error}")))?;
        let robot = document.root_element();
        if !robot.has_tag_name("robot") {
            return Err(model_error("URDF has no <robot> root element".into()));
        }

        let mut joints: [Option<JointLimitConstants>; NUM_JOINTS] = [None; NUM_JOINTS];
        for joint in robot.children().filter(|node| node.has_tag_name("joint")) {
            let Some(name) = joint.attribute("name") else {
                continue;
            };
            let Some(index) = joint_index(name) else {
                continue;
            };
            let position_based = child_element(joint, "position_based_velocity_limits", name)?;
            let limit = child_element(joint, "limit", name)?;
            joints[index] = Some(JointLimitConstants {
                max_velocity: number_attribute(limit, "velocity", name)?,
                velocity_offset: number_attribute(position_based, "velocity_offset", name)?,
                deceleration_limit: number_attribute(position_based, "deceleration_limit", name)?,
                upper_position_limit: number_attribute(limit, "upper", name)?,
                lower_position_limit: number_attribute(limit, "lower", name)?,
            });
        }

        let missing: Vec<String> = (0..NUM_JOINTS)
            .filter(|&index| joints[index].is_none())
            .map(|index| format!("joint{}", index + 1))
            .collect();
        if !missing.is_empty() {
            return Err(model_error(format!("URDF is missing joints: {}", missing.join(" "))));
        }
        Ok(Self {
            joints: joints.map(|joint| joint.expect("all joints checked present")),
        })
    }

    /// Returns each joint's maximum allowed velocity at joint positions `q`.
    pub fn upper(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        std::array::from_fn(|index| {
            let joint = &self.joints[index];
            let braking_velocity =
                (2.0 * joint.deceleration_limit * (joint.upper_position_limit - q[index])).max(0.0).sqrt();
            joint.max_velocity.min((braking_velocity - joint.velocity_offset).max(0.0))
                - JOINT_VELOCITY_LIMITS_TOLERANCE[index]
        })
    }

    /// Returns each joint's minimum allowed (most negative) velocity at joint positions `q`.
    pub fn lower(&self, q: &[f64; NUM_JOINTS]) -> [f64; NUM_JOINTS] {
        std::array::from_fn(|index| {
            let joint = &self.joints[index];
            let braking_velocity =
                (2.0 * joint.deceleration_limit * (q[index] - joint.lower_position_limit)).max(0.0).sqrt();
            (-joint.max_velocity).max((joint.velocity_offset - braking_velocity).min(0.0))
                + JOINT_VELOCITY_LIMITS_TOLERANCE[index]
        })
    }
}

/// Maps a URDF joint name to its index: an exact `jointN` match, or else a name containing `jointN`.
fn joint_index(name: &str) -> Option<usize> {
    let pattern = |index: usize| format!("joint{}", index + 1);
    (0..NUM_JOINTS)
        .find(|&index| name == pattern(index))
        .or_else(|| (0..NUM_JOINTS).find(|&index| name.contains(&pattern(index))))
}

/// Returns the first child element of `joint` named `tag_name`, or an error naming the joint.
fn child_element<'a, 'input>(
    joint: Node<'a, 'input>,
    tag_name: &str,
    joint_name: &str,
) -> FrankaResult<Node<'a, 'input>> {
    joint
        .children()
        .find(|child| child.has_tag_name(tag_name))
        .ok_or_else(|| model_error(format!("missing <{tag_name}> element for joint {joint_name}")))
}

/// Parses attribute `name` of `element` as a number, or returns an error naming the joint.
fn number_attribute(element: Node, name: &str, joint_name: &str) -> FrankaResult<f64> {
    let tag_name = element.tag_name().name();
    let text = element.attribute(name).ok_or_else(|| {
        model_error(format!("missing '{name}' attribute in <{tag_name}> for joint {joint_name}"))
    })?;
    text.trim().parse().map_err(|_| {
        model_error(format!("'{text}' in <{tag_name} {name}> for joint {joint_name} is not a number"))
    })
}

/// Builds a [`FrankaError::Model`] with `message`.
fn model_error(message: String) -> FrankaError {
    FrankaError::Model { message }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rate_limiting::LIMIT_EPS;

    /// The robot URDF used by the model tests.
    const URDF: &str = include_str!("../tests/fixtures/fr3_robot.urdf");

    #[test]
    fn velocity_reaches_zero_at_position_limits() {
        let limits = JointVelocityLimits::from_urdf(URDF).unwrap();
        // Joint 1 limits: lower -2.9007, upper 2.9007.
        let mut q = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0];
        q[0] = 2.9007;
        assert!((limits.upper(&q)[0] - (0.0 - LIMIT_EPS)).abs() < 1e-12);
        q[0] = -2.9007;
        assert!((limits.lower(&q)[0] - (0.0 + LIMIT_EPS)).abs() < 1e-12);
    }

    #[test]
    fn velocity_is_capped_far_from_position_limits() {
        let limits = JointVelocityLimits::from_urdf(URDF).unwrap();
        // Joint 1: max velocity 2.62; at q = 0 the braking term is sqrt(2·6·2.9007) − 0.652 ≈ 5.25.
        let q = [0.0, 0.0, 0.0, -1.5, 0.0, 1.5, 0.0];
        assert!((limits.upper(&q)[0] - (2.62 - LIMIT_EPS)).abs() < 1e-12);
        assert!((limits.lower(&q)[0] - (-2.62 + LIMIT_EPS)).abs() < 1e-12);
    }

    #[test]
    fn rejects_joint_without_position_based_limits() {
        let urdf = URDF.replacen("<position_based_velocity_limits", "<other_element", 1);
        let error = JointVelocityLimits::from_urdf(&urdf).unwrap_err();
        assert!(
            error.to_string().contains("missing <position_based_velocity_limits> element for joint joint1"),
            "{error}"
        );
    }

    #[test]
    fn rejects_missing_joint() {
        let urdf = URDF.replace("name=\"joint7\"", "name=\"wrist\"");
        let error = JointVelocityLimits::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("joint7"), "{error}");
    }
}