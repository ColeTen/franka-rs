//! Construction of a [`KinematicChain`] from a URDF (Unified Robot Description Format) document.

use std::collections::HashMap;

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};
use roxmltree::{Document, Node};

use super::chain::{KinematicChain, RevoluteJoint};
use super::spatial::RigidBodyInertia;
use crate::constants::NUM_JOINTS;
use crate::errors::{FrankaError, FrankaResult};

/// Name of the link whose frame is the arm's flange.
const FLANGE_LINK_NAME: &str = "link8";

/// A `<joint>` element of the URDF.
struct UrdfJoint<'document> {
    /// Value of the `name` attribute.
    name: &'document str,
    /// Value of the `type` attribute.
    joint_type: &'document str,
    /// Name of the parent link.
    parent_link: &'document str,
    /// Name of the child link.
    child_link: &'document str,
    /// Pose of the joint frame in the parent link's frame.
    origin: Isometry3<f64>,
    /// Rotation axis in the joint frame.
    axis: Vector3<f64>,
}

/// Where a link's frame sits in the chain being built.
#[derive(Clone, Copy)]
struct Attachment {
    /// Index of the revolute joint the link moves with, or `None` if fixed to the base.
    joint_index: Option<usize>,
    /// Pose of the link's frame in that joint's frame (or in the base frame).
    placement: Isometry3<f64>,
}

impl KinematicChain {
    /// Builds the chain from a URDF document describing a serial arm of revolute joints.
    ///
    /// Bodies attached through fixed joints are merged into the joint that carries them.
    /// The flange is the frame of the link named `link8`.
    ///
    /// # Errors
    /// Returns [`FrankaError::Model`] if the document is not valid XML, contains a joint type
    /// other than `revolute` or `fixed`, is not a serial chain of exactly seven revolute joints,
    /// or has no `link8` attached to the last joint.
    pub fn from_urdf(urdf: &str) -> FrankaResult<Self> {
        let document = Document::parse(urdf).map_err(|error| model_error(format!("invalid URDF XML: {error}")))?;
        let robot = document.root_element();

        let mut link_inertias = HashMap::new();
        let mut joints = Vec::new();
        for element in robot.children().filter(Node::is_element) {
            match element.tag_name().name() {
                "link" => {
                    let inertia = match child_element(element, "inertial") {
                        Some(inertial) => parse_inertial(inertial)?,
                        None => RigidBodyInertia::zero(),
                    };
                    let name = required_attribute(element, "name")?;
                    if link_inertias.insert(name, inertia).is_some() {
                        return Err(model_error(format!("link '{name}' is defined more than once")));
                    }
                }
                "joint" => joints.push(parse_joint(element)?),
                _ => {}
            }
        }

        let root_link = find_root_link(&link_inertias, &joints)?;
        let mut attachments = HashMap::from([(
            root_link,
            Attachment {
                joint_index: None,
                placement: Isometry3::identity(),
            },
        )]);
        let mut revolute_joints: Vec<RevoluteJoint> = Vec::new();
        let mut links_to_visit = vec![root_link];
        while let Some(parent_link) = links_to_visit.pop() {
            let parent_attachment = attachments[parent_link];
            for joint in joints.iter().filter(|joint| joint.parent_link == parent_link).rev() {
                let child_inertia = *link_inertias.get(joint.child_link).ok_or_else(|| {
                    model_error(format!("joint '{}' references unknown link '{}'", joint.name, joint.child_link))
                })?;
                if attachments.contains_key(joint.child_link) {
                    return Err(model_error(format!(
                        "link '{}' has more than one parent joint",
                        joint.child_link
                    )));
                }
                let placement = parent_attachment.placement * joint.origin;
                let child_attachment = match joint.joint_type {
                    "revolute" => {
                        if parent_attachment.joint_index != revolute_joints.len().checked_sub(1) {
                            return Err(model_error(format!(
                                "joint '{}' branches the chain; only serial chains are supported",
                                joint.name
                            )));
                        }
                        let axis = Unit::try_new(joint.axis, f64::EPSILON).ok_or_else(|| {
                            model_error(format!("joint '{}' has a zero-length axis", joint.name))
                        })?;
                        revolute_joints.push(RevoluteJoint {
                            placement,
                            axis,
                            inertia: child_inertia,
                        });
                        Attachment {
                            joint_index: Some(revolute_joints.len() - 1),
                            placement: Isometry3::identity(),
                        }
                    }
                    "fixed" => {
                        if let Some(joint_index) = parent_attachment.joint_index {
                            let carrier = &mut revolute_joints[joint_index].inertia;
                            *carrier = carrier.combine(&child_inertia.transformed(&placement));
                        }
                        Attachment {
                            joint_index: parent_attachment.joint_index,
                            placement,
                        }
                    }
                    other => {
                        return Err(model_error(format!(
                            "joint '{}' has unsupported type '{other}'",
                            joint.name
                        )));
                    }
                };
                attachments.insert(joint.child_link, child_attachment);
                links_to_visit.push(joint.child_link);
            }
        }

        let joint_count = revolute_joints.len();
        let joints: [RevoluteJoint; NUM_JOINTS] = revolute_joints.try_into().map_err(|_| {
            model_error(format!("expected {NUM_JOINTS} revolute joints, found {joint_count}"))
        })?;
        let flange = match attachments.get(FLANGE_LINK_NAME) {
            Some(Attachment {
                joint_index: Some(joint_index),
                placement,
            }) if *joint_index == NUM_JOINTS - 1 => *placement,
            _ => {
                return Err(model_error(format!(
                    "link '{FLANGE_LINK_NAME}' must be rigidly attached to the last revolute joint"
                )));
            }
        };
        Ok(Self { joints, flange })
    }
}

/// Builds a [`FrankaError::Model`] with `message`.
fn model_error(message: String) -> FrankaError {
    FrankaError::Model { message }
}

/// Returns the first child element of `node` named `tag_name`.
fn child_element<'a, 'input>(node: Node<'a, 'input>, tag_name: &str) -> Option<Node<'a, 'input>> {
    node.children().find(|child| child.has_tag_name(tag_name))
}

/// Returns the value of attribute `name` on `node`, or an error if it is missing.
fn required_attribute<'a>(node: Node<'a, '_>, name: &str) -> FrankaResult<&'a str> {
    node.attribute(name).ok_or_else(|| {
        model_error(format!("<{}> is missing attribute '{name}'", node.tag_name().name()))
    })
}

/// Parses a finite floating-point attribute value.
fn parse_number(text: &str) -> FrankaResult<f64> {
    text.trim()
        .parse()
        .ok()
        .filter(|value: &f64| value.is_finite())
        .ok_or_else(|| model_error(format!("'{text}' is not a finite number")))
}

/// Parses a space-separated triple such as `"0 0 0.333"`, or returns `default` if absent.
fn parse_triple(text: Option<&str>, default: Vector3<f64>) -> FrankaResult<Vector3<f64>> {
    let Some(text) = text else {
        return Ok(default);
    };
    let values = text
        .split_whitespace()
        .map(parse_number)
        .collect::<FrankaResult<Vec<f64>>>()?;
    match values.as_slice() {
        [x, y, z] => Ok(Vector3::new(*x, *y, *z)),
        _ => Err(model_error(format!("'{text}' is not a triple of numbers"))),
    }
}

/// Parses an optional `<origin xyz rpy>` element into a pose; absent values default to zero.
///
/// The rotation is `Rz(yaw) · Ry(pitch) · Rx(roll)` for `rpy = "roll pitch yaw"`.
fn parse_origin(origin: Option<Node>) -> FrankaResult<Isometry3<f64>> {
    let translation = parse_triple(origin.and_then(|node| node.attribute("xyz")), Vector3::zeros())?;
    let rpy = parse_triple(origin.and_then(|node| node.attribute("rpy")), Vector3::zeros())?;
    let rotation = Rotation3::from_axis_angle(&Vector3::z_axis(), rpy.z)
        * Rotation3::from_axis_angle(&Vector3::y_axis(), rpy.y)
        * Rotation3::from_axis_angle(&Vector3::x_axis(), rpy.x);
    Ok(Isometry3::from_parts(
        Translation3::from(translation),
        UnitQuaternion::from_rotation_matrix(&rotation),
    ))
}

/// Parses an `<inertial>` element into an inertia expressed in its link's frame.
fn parse_inertial(inertial: Node) -> FrankaResult<RigidBodyInertia> {
    let origin = parse_origin(child_element(inertial, "origin"))?;
    let mass_element = child_element(inertial, "mass")
        .ok_or_else(|| model_error("<inertial> is missing <mass>".into()))?;
    let mass = parse_number(required_attribute(mass_element, "value")?)?;
    let inertia_element = child_element(inertial, "inertia")
        .ok_or_else(|| model_error("<inertial> is missing <inertia>".into()))?;
    let component = |name| parse_number(required_attribute(inertia_element, name)?);
    let (ixx, ixy, ixz) = (component("ixx")?, component("ixy")?, component("ixz")?);
    let (iyy, iyz, izz) = (component("iyy")?, component("iyz")?, component("izz")?);
    let inertia_in_origin_axes = Matrix3::new(ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz);
    let rotation = origin.rotation.to_rotation_matrix();
    Ok(RigidBodyInertia::new(
        mass,
        origin.translation.vector,
        rotation * inertia_in_origin_axes * rotation.transpose(),
    ))
}

/// Parses a `<joint>` element.
fn parse_joint<'document>(joint: Node<'document, '_>) -> FrankaResult<UrdfJoint<'document>> {
    let link_name = |tag_name| {
        child_element(joint, tag_name)
            .ok_or_else(|| model_error(format!("<joint> is missing <{tag_name}>")))
            .and_then(|element| required_attribute(element, "link"))
    };
    Ok(UrdfJoint {
        name: required_attribute(joint, "name")?,
        joint_type: required_attribute(joint, "type")?,
        parent_link: link_name("parent")?,
        child_link: link_name("child")?,
        origin: parse_origin(child_element(joint, "origin"))?,
        axis: parse_triple(
            child_element(joint, "axis").and_then(|axis| axis.attribute("xyz")),
            Vector3::x(),
        )?,
    })
}

/// Returns the single link that is not the child of any joint.
fn find_root_link<'document>(
    link_inertias: &HashMap<&'document str, RigidBodyInertia>,
    joints: &[UrdfJoint<'document>],
) -> FrankaResult<&'document str> {
    let mut roots = link_inertias
        .keys()
        .copied()
        .filter(|link| joints.iter().all(|joint| joint.child_link != *link));
    match (roots.next(), roots.next()) {
        (Some(root), None) => Ok(root),
        _ => Err(model_error("URDF must have exactly one root link".into())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two-link URDF used to check rejection rules.
    fn urdf_with(joints: &str, links: &str) -> String {
        format!(r#"<robot name="test"><link name="base"/>{links}{joints}</robot>"#)
    }

    #[test]
    fn rejects_wrong_joint_count() {
        let urdf = urdf_with(
            r#"<joint name="j1" type="revolute"><parent link="base"/><child link="link1"/></joint>
               <joint name="j8" type="fixed"><parent link="link1"/><child link="link8"/></joint>"#,
            r#"<link name="link1"/><link name="link8"/>"#,
        );
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("expected 7 revolute joints"), "{error}");
    }

    #[test]
    fn rejects_unsupported_joint_type() {
        let urdf = urdf_with(
            r#"<joint name="j1" type="prismatic"><parent link="base"/><child link="link1"/></joint>"#,
            r#"<link name="link1"/>"#,
        );
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("unsupported type"), "{error}");
    }

    #[test]
    fn rejects_branching_chain() {
        let urdf = urdf_with(
            r#"<joint name="a" type="revolute"><parent link="base"/><child link="la"/></joint>
               <joint name="b" type="revolute"><parent link="base"/><child link="lb"/></joint>"#,
            r#"<link name="la"/><link name="lb"/>"#,
        );
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("branches"), "{error}");
    }

    #[test]
    fn rejects_link_with_two_parents() {
        let urdf = urdf_with(
            r#"<joint name="a" type="fixed"><parent link="base"/><child link="la"/></joint>
               <joint name="b" type="fixed"><parent link="la"/><child link="lb"/></joint>
               <joint name="c" type="fixed"><parent link="lb"/><child link="la"/></joint>"#,
            r#"<link name="la"/><link name="lb"/>"#,
        );
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("more than one parent"), "{error}");
    }

    #[test]
    fn rejects_duplicate_link() {
        let urdf = urdf_with("", r#"<link name="base"/>"#);
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("more than once"), "{error}");
    }

    #[test]
    fn rejects_zero_axis() {
        let urdf = urdf_with(
            r#"<joint name="j1" type="revolute"><parent link="base"/><child link="link1"/><axis xyz="0 0 0"/></joint>"#,
            r#"<link name="link1"/>"#,
        );
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("zero-length axis"), "{error}");
    }

    #[test]
    fn rejects_non_finite_number() {
        let urdf = urdf_with(
            r#"<joint name="j1" type="revolute"><parent link="base"/><child link="link1"/><origin xyz="0 nan 0"/></joint>"#,
            r#"<link name="link1"/>"#,
        );
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("finite"), "{error}");
    }

    #[test]
    fn rejects_missing_flange() {
        let urdf = include_str!("../../external/libfranka/test/fr3.urdf").replace("link8", "link9");
        let error = KinematicChain::from_urdf(&urdf).unwrap_err();
        assert!(error.to_string().contains("link8"), "{error}");
    }

    #[test]
    fn parses_fr3_geometry() {
        let chain = KinematicChain::from_urdf(include_str!("../../external/libfranka/test/fr3.urdf")).unwrap();
        assert!((chain.joints[3].placement.translation.vector - Vector3::new(0.0825, 0.0, 0.0)).norm() < 1e-12);
        assert!((chain.joints[6].placement.translation.vector - Vector3::new(0.088, 0.0, 0.0)).norm() < 1e-12);
        assert!((chain.flange.translation.vector - Vector3::new(0.0, 0.0, 0.107)).norm() < 1e-12);
        assert!((chain.joints[0].inertia.mass - 2.9274653454).abs() < 1e-12);
    }
}
