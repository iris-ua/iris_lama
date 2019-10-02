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

#include "lama/image.h"
#include "lama/image_io.h"

#include "lama/sdm/export.h"

#include "lama/sdm/occupancy_map.h"
#include "lama/sdm/distance_map.h"

namespace {

void build_image(const lama::OccupancyMap& occ, lama::Image& image, double zed)
{
    Eigen::Vector3ui min, max;
    occ.bounds(min,max);

    Eigen::Vector3ui dim = max - min;

    min(2) = occ.w2m(Eigen::Vector3d(0,0,zed)).z();

    image.alloc(dim(0), dim(1), 1);
    image.fill(90);

    occ.visit_all_cells([&image, &occ, &min](const Eigen::Vector3ui& coords){
        // Filter by z (zed).
        if (occ.is_3d and coords(2) != min(2))
            return;

        Eigen::Vector3ui adj_coords = coords - min;

        if (occ.isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 255;
        else if (occ.isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else
            image(adj_coords(0), adj_coords(1)) = 127;
    });
}

void build_image(const lama::DistanceMap& dm, lama::Image& image, double zed)
{
    Eigen::Vector3ui min, max;
    dm.bounds(min,max);

    Eigen::Vector3ui dim = max - min;

    min(2) = dm.w2m(Eigen::Vector3d(0,0,zed)).z();

    image.alloc(dim(0), dim(1), 1);
    image.fill(127);

    dm.visit_all_cells([&image, &dm, &min](const Eigen::Vector3ui& coords){
        // Filter by z_slice.
        if (dm.is_3d and coords(2) != min(2))
            return;

        Eigen::Vector3ui adj_coords = coords - min;
        image(adj_coords(0), adj_coords(1)) = dm.distance(coords) * 255 / dm.maxDistance();
    });
}

} // namespace

bool lama::sdm::export_to_png(const OccupancyMap& occ, const std::string& filename, double zed)
{
    Image image;
    build_image(occ, image, zed);
    return image_write_png(image, filename);
}

bool lama::sdm::export_to_png(const DistanceMap& dm, const std::string& filename, double zed)
{
    Image image;
    build_image(dm, image, zed);
    return image_write_png(image, filename);
}

