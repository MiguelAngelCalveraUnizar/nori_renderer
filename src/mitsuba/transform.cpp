/*
    This file is part of Mitsuba, a physically based rendering system.

    Copyright (c) 2007-2014 by Wenzel Jakob and others.

    Mitsuba is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Mitsuba is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.

    This file was edited to fit inside Nori, please go to the original
    for a good way of doing things, don't take this one as the example
*/
#include <nori/transform.h>
#include <nori/frame.h>

NORI_NAMESPACE_BEGIN

// Mitsuba stuff:
Transform Transform::scale(const Vector3f& v) {
    Eigen::Matrix4f trafo = Eigen::Matrix4f::Identity();
    trafo(0, 0) = v[0];
    trafo(1, 1) = v[1];
    trafo(2, 2) = v[2];

    Eigen::Matrix4f invTrafo = Eigen::Matrix4f::Identity();
    invTrafo(0, 0) = 1 / v[0];
    invTrafo(1, 1) = 1 / v[1];
    invTrafo(2, 2) = 1 / v[2];

    return Transform(trafo, invTrafo);
}

Transform Transform::translate(const Vector3f& v) {
    Eigen::Matrix4f trafo = Eigen::Matrix4f::Identity();
    trafo(0, 3) = v[0];
    trafo(1, 3) = v[1];
    trafo(2, 3) = v[2];

    Eigen::Matrix4f invTrafo = Eigen::Matrix4f::Identity();
    invTrafo(0, 3) = -v[0];
    invTrafo(1, 3) = -v[1];
    invTrafo(2, 3) = -v[2];

    return Transform(trafo, invTrafo);
}

NORI_NAMESPACE_END
