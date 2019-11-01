/*
 * IRIS Localization and Mapping (LaMa)
 *
 * Copyright (c) 2019-today, Eurico Pedrosa, University of Aveiro - Portugal
 * All rights reserved.
 * License: New BSD
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Aveiro nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "lama/sdm/truncated_signed_distance_map.h"
#include "lama/sdm/marching_cubes.h"

lama::TruncatedSignedDistanceMap::TruncatedSignedDistanceMap(double resolution, uint32_t patch_size, bool is3d)
    : DistanceMap(resolution, sizeof(tsd_t), patch_size, is3d)
{
    maximum_weight_ = 10000;
    truncate_size_  = 0.15;

    epsilon_ = resolution;
    delta_   = 4 * resolution;
}

lama::TruncatedSignedDistanceMap::TruncatedSignedDistanceMap(const TruncatedSignedDistanceMap& other)
    : DistanceMap(other)
{
    maximum_weight_ = other.maximum_weight_;
    truncate_size_  = other.truncate_size_;
}


lama::TruncatedSignedDistanceMap::~TruncatedSignedDistanceMap()
{}


double lama::TruncatedSignedDistanceMap::distance(const Vector3d& coordinates, Vector3d* gradient) const
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

double lama::TruncatedSignedDistanceMap::distance(const Vector3ui& coordinates) const
{
    const tsd_t* cell = (const tsd_t*)get(coordinates);
    if (cell == 0 || cell->weight == 0.0)
        return truncate_size_;

    return cell->distance;
}

size_t lama::TruncatedSignedDistanceMap::insertPointCloud(const PointCloudXYZ::Ptr& cloud)
{
    Eigen::Affine3d affine = Eigen::Translation3d(cloud->sensor_origin_) * cloud->sensor_orientation_;

    KeySet hit_points;
    const size_t num_points = cloud->points.size();

    for (size_t i = 0; i < num_points; ++i){
        Vector3d hit = affine * cloud->points[i];

        std::pair<typename KeySet::iterator,bool> ret = hit_points.insert(this->w2m(hit));
        if (ret.second == false) continue;

        integrate(cloud->sensor_origin_, hit);
    }// end for

    return hit_points.size();
}


void lama::TruncatedSignedDistanceMap::integrate(const Vector3d& origin, const Vector3d& hit)
{

    Vector3d direction_vector = hit - origin;
    float squared_norm = direction_vector.squaredNorm();
    direction_vector.normalize();

    float truncate = std::min(squared_norm, truncate_size_);

    Vector3ui start = w2m(hit - direction_vector * truncate);
    Vector3ui end   = w2m(hit + direction_vector * truncate_size_);

    VectorVector3ui voxels_coordinates;
    computeRay(start, end, voxels_coordinates);

    // cache the constants
    const float inv_squared_norm = 1.0 / squared_norm;
    const float inv_delta_less_epsilon = 1.0 / (delta_ - epsilon_);

    VectorVector3ui::const_iterator it = voxels_coordinates.begin();
    VectorVector3ui::const_iterator it_end = voxels_coordinates.end();
    for (; it != it_end; ++it){
        // Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning
        // Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan
        // IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2017

        tsd_t * cell = (tsd_t*)get(*it);

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

        //float previous_distance = cell->distance;
        cell->distance = (cell->weight * cell->distance + weight * distance) / (cell->weight + weight);
        cell->weight = std::min(cell->weight + weight, maximum_weight_);

    }// end for(iterator)
}

void lama::TruncatedSignedDistanceMap::setMaxDistance(double distance)
{
    truncate_size_ = distance;
}

double lama::TruncatedSignedDistanceMap::maxDistance() const
{
    return truncate_size_;
}

void lama::TruncatedSignedDistanceMap::toMesh(PolygonMesh& mesh) const
{
    Array<Vector3ui, 8> delta = {
        Vector3ui{0, 0, 0}, Vector3ui{1, 0, 0}, Vector3ui{1, 1, 0}, Vector3ui{0, 1, 0},
        Vector3ui{0, 0, 1}, Vector3ui{1, 0, 1}, Vector3ui{1, 1, 1}, Vector3ui{0, 1, 1}
    };

    visit_all_cells([&delta, &mesh, this](const Vector3ui& current_cell){
        MarchingCubes::VertexArray vertex_coords;
        MarchingCubes::SDFArray    sdf;

        bool valid_neighbours = true;
        for (int i = 0; i < 8; ++i){
            Vector3ui coords = current_cell + delta[i];

            const tsd_t* cell = (const tsd_t*) this->get(coords);
            if (cell == nullptr || cell->weight == 0.0){
                valid_neighbours = false;
                break;
            }

            vertex_coords[i] = this->m2w(coords).cast<float>();
            sdf[i] = distance(coords);
        }

        if (valid_neighbours == false)
            return;

        MarchingCubes::EdgeArray edge_coords;
        MarchingCubes::interpolate_edge_vertices(vertex_coords, sdf, edge_coords);

        int index   = MarchingCubes::calculate_vertex_configuration(sdf);
        int* tt_row = MarchingCubes::triangle_table[index];

        int col = 0;
        while ((index = tt_row[col]) != -1){

            mesh.vertex.push_back(edge_coords[index]);
            mesh.index.push_back(mesh.vertex.size()-1);
            index = tt_row[col + 1];

            mesh.vertex.push_back(edge_coords[index]);
            mesh.index.push_back(mesh.vertex.size()-1);
            index = tt_row[col + 2];

            mesh.vertex.push_back(edge_coords[index]);
            mesh.index.push_back(mesh.vertex.size()-1);
            col += 3;

        }// end while
    });

}

