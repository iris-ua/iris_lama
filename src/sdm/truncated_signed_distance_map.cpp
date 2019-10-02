//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#include <rtk/sdm/truncated_signed_distance_map.h>

namespace rtk {
namespace sdm {

TruncatedSignedDistanceMap::TruncatedSignedDistanceMap(double resolution, uint32_t patch_size, bool is3d)
    : DistanceMap(resolution, sizeof(truncated_signed_distance_t), patch_size, is3d)
{
    maximum_weight_ = 10000;
    truncate_size_  = 0.15;

    epsilon_ = resolution;
    delta_   = 4 * resolution;
}

TruncatedSignedDistanceMap::TruncatedSignedDistanceMap(const TruncatedSignedDistanceMap& other)
    : DistanceMap(other)
{
    maximum_weight_ = other.maximum_weight_;
    truncate_size_  = other.truncate_size_;
}


TruncatedSignedDistanceMap::~TruncatedSignedDistanceMap()
{}


double TruncatedSignedDistanceMap::distance(const Vector3d& coordinates, Vector3d* gradient) const
{
    Vector3d map_coords = w2m_nocast(coordinates);
    Vector3ui disc_coords(map_coords.cast<unsigned int>());

    Vector3d mu(map_coords - disc_coords.cast<double>());
    Vector3d muinv = Vector3d::Ones() - mu;

    double dist = 0;

    if ( not is_3d ){
        Matrix<double, 4, 1> val;
        val << distance(disc_coords),
            distance(Vector3ui(disc_coords + Vector3ui(1, 0, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 1, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 1, 0)));

        dist = val(0) * muinv(0) * muinv(1) +
            val(1) * muinv(1) * mu(0) +
            val(2) * muinv(0) * mu(1) +
            val(3) * mu(0) * mu(1);

        if (gradient)
            *gradient << -((val(0) - val(1)) * muinv(1) + (val(2) - val(3))*mu(1)) * scale,
                         -((val(0) - val(2)) * muinv(0) + (val(1) - val(3))*mu(0)) * scale,
                          0;

    } else {
        Matrix<double, 8, 1> val;
        val <<  distance(disc_coords),
            distance(Vector3ui(disc_coords + Vector3ui(1, 0, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 1, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 1, 0))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 0, 1))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 0, 1))),
            distance(Vector3ui(disc_coords + Vector3ui(0, 1, 1))),
            distance(Vector3ui(disc_coords + Vector3ui(1, 1, 1)));

        dist = val(0) * muinv.prod() +                  // V000
            val(1) * mu(0) * muinv(1) * muinv(2) +      // V100
            val(2) * muinv(0) * mu(1) * muinv(2) +      // V010
            val(3) * mu(0) * mu(1) * muinv(2) +         // V110
            val(4) * muinv(0) * muinv(1) * mu(2) +      // V001
            val(5) * mu(0) * muinv(1) * mu(2) +         // V101
            val(6) * muinv(0) * mu(1) * mu(2) +         // V011
            val(7) * mu.prod();                         // V111

        if (gradient){

            double a, b;
            // X gradient
            a = (val(0) - val(1)) * muinv(1) + (val(2) - val(3)) * mu(1);
            b = (val(4) - val(5)) * muinv(1) + (val(6) - val(7)) * mu(1);
            (*gradient)[0] = -(a * muinv(2) + b * mu(2)) * scale;

            // ------------------
            // Y gradient
            a = (val(0) - val(2)) * muinv(0) + (val(1) - val(3)) * mu(0);
            b = (val(4) - val(6)) * muinv(0) + (val(5) - val(7)) * mu(0);
            (*gradient)[1] = -(a * muinv(2) + b * mu(2)) * scale;

            // ------------------
            // Z gradient
            a = (val(0) - val(4)) * muinv(0) + (val(1) - val(5)) * mu(0);
            b = (val(2) - val(6)) * muinv(0) + (val(3) - val(7)) * mu(0);
            (*gradient)[2] = -(a * muinv(1) + b * mu(1)) * scale;

        }
    }

    return dist;
}

double TruncatedSignedDistanceMap::distance(const Vector3ui& coordinates) const
{
    const truncated_signed_distance_t* cell = (const truncated_signed_distance_t*)get(coordinates);
    if (cell == 0 || cell->weight == 0.0)
        return truncate_size_;

    return cell->distance;
}

size_t TruncatedSignedDistanceMap::insertPointCloud(const PointCloudXYZ::Ptr& cloud)
{
    Eigen::Affine3d affine = Eigen::Translation3d(cloud->sensor_origin_) * cloud->sensor_orientation_;

    KeySet hit_points;
    const size_t num_points = cloud->points.size();

    for (int i = 0; i < num_points; ++i){
        Vector3d hit = affine * cloud->points[i];

        std::pair<typename KeySet::iterator,bool> ret = hit_points.insert(this->w2m(hit));
        if (ret.second == false) continue;

        integrate(cloud->sensor_origin_, hit);
    }// end for

    return hit_points.size();
}


void TruncatedSignedDistanceMap::integrate(const Vector3d& origin, const Vector3d& hit)
{
    Vector3d direction_vector = hit - origin;
    float squared_norm = direction_vector.squaredNorm();
    direction_vector.normalize();

    float truncate = std::min(squared_norm, truncate_size_);

    Vector3ui start = w2m(hit - direction_vector * truncate_size_);
    Vector3ui end   = w2m(hit + direction_vector * truncate_size_);

    VectorVector3ui voxels_coordinates;
    computeRay(start, end, voxels_coordinates);

    // cache the constants
    const float inv_squared_norm = 1.0 / squared_norm;
    const float inv_delta_less_epsilon = 1.0 / (delta_ - epsilon_);

    VectorVector3ui::const_iterator it = voxels_coordinates.begin();
    VectorVector3ui::const_iterator it_end = voxels_coordinates.end();
    for (; it != it_end; ++it){

        truncated_signed_distance_t* cell = (truncated_signed_distance_t*)get(*it);

        Vector3d voxel_center = m2w(*it);
        Vector3d origin_to_hit  = hit - origin;
        Vector3d voxel_center_to_hit  = hit - voxel_center;

        float distance = voxel_center_to_hit.norm() * sign( voxel_center_to_hit.dot(origin_to_hit) );
        float weight;

        if (distance < -delta_)
            continue; //weight = 0;
        else if (-delta_ <= distance and distance <= -epsilon_)
            weight = (distance + delta_) * inv_squared_norm * inv_delta_less_epsilon;
        else
            weight = inv_squared_norm;

        float previous_distance = cell->distance;
        cell->distance = (cell->weight * cell->distance + weight * distance) / (cell->weight + weight);
        cell->weight = std::min(cell->weight + weight, maximum_weight_);

    }// end for(iterator)
}

void TruncatedSignedDistanceMap::setMaxDistance(double distance)
{
    truncate_size_ = distance;
}

double TruncatedSignedDistanceMap::maxDistance() const
{
    return truncate_size_;
}

}} /* rtk::sdm */
