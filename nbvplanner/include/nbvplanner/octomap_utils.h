#pragma once
#include <Eigen/Core>

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>


namespace octoutils {

using namespace Eigen;
enum CellStatus{
    kUnknown,
    kFree,
    kOccupied
};

// Convenience functions for octomap point <-> eigen conversions.
static octomap::point3d pointEigenToOctomap(const Eigen::Vector3d& point) {
    return octomap::point3d(point.x(), point.y(), point.z());
}
static Eigen::Vector3d pointOctomapToEigen(const octomap::point3d& point) {
    return Eigen::Vector3d(point.x(), point.y(), point.z());
}
static
CellStatus getLineStatus(const Vector3d &start, const Vector3d &end, std::shared_ptr<octomap::OcTree> &ot_)
{

    // Get all node keys for this line.
    // This is actually a typedef for a vector of OcTreeKeys.
    // Can't use the key_ray_ temp member here because this is a const function.
    octomap::KeyRay key_ray;
    ot_->computeRayKeys(pointEigenToOctomap(start),pointEigenToOctomap(end),
                        key_ray);

    // Now check if there are any unknown or occupied nodes in the ray.
    for (octomap::OcTreeKey key : key_ray) {
        octomap::OcTreeNode* node = ot_->search(key);
        if (node == NULL) {
            return CellStatus::kOccupied;

        } else if (ot_->isNodeOccupied(node)) {
            return CellStatus::kOccupied;
        }
    }
    return CellStatus::kFree;

};


static CellStatus getLineStatusBoundingBox(const Vector3d &start, const Vector3d &end, const Vector3d &bounding_box_size, std::shared_ptr<octomap::OcTree> &ot_)
{
    const double epsilon = 0.001;  // Small offset
    CellStatus ret = CellStatus::kFree;
    const double& resolution = ot_->getResolution();

    // Check corner connections and depending on resolution also interior:
    // Discretization step is smaller than the octomap resolution, as this way
    // no cell can possibly be missed
    double x_disc = bounding_box_size.x() /
            ceil((bounding_box_size.x() + epsilon) / resolution);
    double y_disc = bounding_box_size.y() /
            ceil((bounding_box_size.y() + epsilon) / resolution);
    double z_disc = bounding_box_size.z() /
            ceil((bounding_box_size.z() + epsilon) / resolution);

    // Ensure that resolution is not infinit
    if (x_disc <= 0.0) x_disc = 1.0;
    if (y_disc <= 0.0) y_disc = 1.0;
    if (z_disc <= 0.0) z_disc = 1.0;

    const Eigen::Vector3d bounding_box_half_size = bounding_box_size * 0.5;

    for (double x = -bounding_box_half_size.x(); x <= bounding_box_half_size.x();
         x += x_disc) {
        for (double y = -bounding_box_half_size.y();
             y <= bounding_box_half_size.y(); y += y_disc) {
            for (double z = -bounding_box_half_size.z();
                 z <= bounding_box_half_size.z(); z += z_disc) {
                Eigen::Vector3d offset(x, y, z);
                ret = getLineStatus(start + offset, end + offset, ot_);
                if (ret != CellStatus::kFree) {
                    return ret;
                }
            }
        }
    }
      return CellStatus::kFree;
}

static CellStatus getVisibility(
        const Eigen::Vector3d& view_point, const Eigen::Vector3d& voxel_to_test,
        bool stop_at_unknown_cell,std::shared_ptr<octomap::OcTree> &ot) {
    // Get all node keys for this line.
    // This is actually a typedef for a vector of OcTreeKeys.
    // Can't use the key_ray_ temp member here because this is a const function.
    octomap::KeyRay key_ray;

    ot->computeRayKeys(pointEigenToOctomap(view_point),
                       pointEigenToOctomap(voxel_to_test), key_ray);

    const octomap::OcTreeKey& voxel_to_test_key =
            ot->coordToKey(pointEigenToOctomap(voxel_to_test));

    // Now check if there are any unknown or occupied nodes in the ray,
    // except for the voxel_to_test key.
    for (octomap::OcTreeKey key : key_ray) {
        if (key != voxel_to_test_key) {
            octomap::OcTreeNode* node = ot->search(key);
            if (node == NULL) {
                if (stop_at_unknown_cell) {
                    return CellStatus::kUnknown;
                }
            } else if (ot->isNodeOccupied(node)) {
                return CellStatus::kOccupied;
            }
        }
    }
    return CellStatus::kFree;
}


static CellStatus getCellProbabilityPoint(
        const Eigen::Vector3d& point, double* probability,std::shared_ptr<octomap::OcTree> &ot)  {
    octomap::OcTreeNode* node = ot->search(point.x(), point.y(), point.z());
    if (node == NULL) {
        if (probability) {
            *probability = -1.0;
        }
        return CellStatus::kUnknown;
    } else {
        if (probability) {
            *probability = node->getOccupancy();
        }
        if (ot->isNodeOccupied(node)) {
            return CellStatus::kOccupied;
        } else {
            return CellStatus::kFree;
        }
    }
}

};
